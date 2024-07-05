//
// Created by qin on 2/24/24.
//

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <librealsense2/rs.hpp> // Include Intel RealSense Cross Platform API

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // Declare the RealSense pipeline, encapsulating the actual device and sensors
    std::unique_ptr<rs2::pipeline> pipe;
    pipe.reset(new rs2::pipeline());

    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    // Start streaming with the default recommended configuration
    rs2::pipeline_profile profile = pipe->start(cfg);

    // Set the white balance and exposure to auto to get good values
    for (rs2::sensor &sensor: profile.get_device().query_sensors()) {
        if (sensor.get_stream_profiles()[0].stream_type() == RS2_STREAM_COLOR) {
            sensor.set_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, true);
            sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, true);
        }
    }

    // Determine the depth scale
    float depth_scale = std::numeric_limits<float>::quiet_NaN();
    for (rs2::sensor &sensor: profile.get_device().query_sensors()) {
        // Check if the sensor is a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>()) {
            depth_scale = dpt.get_depth_scale();
        }
    }
    if (std::isnan(depth_scale)) {
        std::cout << "Cannot determine the depth scale" << std::endl;
    }
    float depth_scaling = 1.0 / depth_scale;
    std::cout << "Depth scaling: " << depth_scaling << std::endl;

    // Find streams
    rs2_stream color_stream = RS2_STREAM_ANY;
    rs2_intrinsics color_intrinsics;

    rs2_stream depth_stream = RS2_STREAM_ANY;
    rs2_intrinsics depth_intrinsics;

    for (rs2::stream_profile sp: profile.get_streams()) {
        rs2_stream profile_stream = sp.stream_type();
        if (color_stream == RS2_STREAM_ANY && profile_stream == RS2_STREAM_COLOR) {
            color_stream = profile_stream;
            color_intrinsics = sp.as<rs2::video_stream_profile>().get_intrinsics();
        }
        if (depth_stream == RS2_STREAM_ANY && profile_stream == RS2_STREAM_DEPTH) {
            depth_stream = profile_stream;
            depth_intrinsics = sp.as<rs2::video_stream_profile>().get_intrinsics();
        }
    }
    if (color_stream == RS2_STREAM_ANY) {
        std::cout << "Cannot find a color stream" << std::endl;
    }
    if (depth_stream == RS2_STREAM_ANY) {
        std::cout << "Cannot find a depth stream" << std::endl;
    }

    // Prepare depth reprojection to the color stream
    std::shared_ptr<rs2::align> align_;
    align_.reset(new rs2::align(color_stream));

    // Set color camera
    // TODO: Ignoring the color stream's distortion. In principle, we should undistort the
    //       frames ourselves (and possibly create undistorted rs2::video_frames such
    //       that we can still use rs2::align to reproject the depth images to the
    //       color frame?). In practice, the distortion coefficients for my camera were all
    //       zero, so we can simply ignore them in that case.
//    float color_parameters[4];
//    color_parameters[0] = color_intrinsics.fx;
//    color_parameters[1] = color_intrinsics.fy;
//    color_parameters[2] = color_intrinsics.ppx + 0.5f;
//    color_parameters[3] = color_intrinsics.ppy + 0.5f;
    if (color_intrinsics.coeffs[0] != 0 ||
        color_intrinsics.coeffs[1] != 0 ||
        color_intrinsics.coeffs[2] != 0 ||
        color_intrinsics.coeffs[3] != 0 ||
        color_intrinsics.coeffs[4] != 0) {
        std::cout << "Ignoring the color stream's distortion, but at least one of the distortion coefficients is non-zero!"
                  << std::endl;
        std::cout << "Model: " << color_intrinsics.model << ". Coefficients: "
                  << color_intrinsics.coeffs[0] << ", "
                  << color_intrinsics.coeffs[1] << ", "
                  << color_intrinsics.coeffs[2] << ", "
                  << color_intrinsics.coeffs[3] << ", "
                  << color_intrinsics.coeffs[4] << "" << std::endl;
    }

    // Wait a short time to let the auto-exposure and white balance find a good initial setting
    for (int i = 0; i < 30; ++ i) {
        pipe->wait_for_frames();
    }

    // Set the white balance and exposure to fixed for more consistent coloring of the reconstruction
    for (rs2::sensor& sensor : profile.get_device().query_sensors()) {
        if (sensor.get_stream_profiles()[0].stream_type() == RS2_STREAM_COLOR) {
            sensor.set_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, false);
            sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, false);
        }
    }

    // Wait some more to let the new settings take effect
    for (int i = 0; i < 15; ++ i) {
        pipe->wait_for_frames();
    }

    //image_transport will publish the video that can be compressed
    auto g_node = rclcpp::Node::make_shared("pub_rgbd_aligned_node");
    // set depth image format parameter "camera.depth.image_rect_raw.format" to "png"
    g_node->declare_parameter<std::string>("camera.depth.image_rect_raw.format", "png");
    // image_transport will publish the video that can be compressed
    image_transport::ImageTransport it(g_node);
    image_transport::Publisher pub_color = it.advertise("/camera/color/image_raw", 10);
    image_transport::Publisher pub_depth = it.advertise("/camera/depth/image_rect_raw", 10);
    // Publisher for camera info
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_camera_info =
            g_node->create_publisher<sensor_msgs::msg::CameraInfo>("/D435/camera_info", 10);

    cv::Mat color_cv, depth_cv;

    while (rclcpp::ok()) {
        rs2::frameset data = pipe->wait_for_frames(); // Wait for next set of frames from the camera (must be called in a loop)
        std_msgs::msg::Header header;
        header.stamp = rclcpp::Clock().now();

//    rs2::frame color_frame = color_map.colorize(data.get_color_frame());
        rs2::frame color_frame = data.get_color_frame();
        rs2::frame depth_frame = data.get_depth_frame();

        // Reproject the depth image into the color frame
        auto processed = align_->process(data);
        // rs2::video_frame other_frame = processed.first(align_to);
        rs2::depth_frame aligned_depth = processed.get_depth_frame();        // set header time stamp

        // Query frame size (width and height)
//        const int w = depth_frame.as<rs2::video_frame>().get_width();
//        const int h = depth_frame.as<rs2::video_frame>().get_height();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        color_cv = cv::Mat(cv::Size(640, 480), CV_8UC3, (void *) color_frame.get_data(), cv::Mat::AUTO_STEP);
        depth_cv = cv::Mat(cv::Size(640, 480), CV_16UC1, (void *) aligned_depth.get_data(), cv::Mat::AUTO_STEP);

//        cv::imshow("D435/color", color_cv);
//        cv::imshow("D435/depth", depth_cv);
//        cv::waitKey(1);

//    depth_cv = cv::Mat(cv::Size(640, 480), CV_8UC3, (void *) depth_frame.get_data(), cv::Mat::AUTO_STEP);
        pub_color.publish(cv_bridge::CvImage(header, "rgb8", color_cv).toImageMsg());
        pub_depth.publish(cv_bridge::CvImage(header, "mono16", depth_cv).toImageMsg());

        // pubish camera info
        sensor_msgs::msg::CameraInfo camera_info_msg;
        camera_info_msg.header.frame_id = "D435";
        camera_info_msg.width = color_intrinsics.width;
        camera_info_msg.height = color_intrinsics.height;
        camera_info_msg.k = {color_intrinsics.fx, 0, color_intrinsics.ppx,
                             0, color_intrinsics.fy, color_intrinsics.ppy,
                             0, 0, 1};
        pub_camera_info->publish(camera_info_msg);

        rclcpp::spin_some(g_node);
    }
}