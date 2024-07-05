#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <librealsense2/rs.hpp> // Include Intel RealSense Cross Platform API

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // Declare the RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    // Start streaming with the default recommended configuration
    pipe.start(cfg);

    auto g_node = rclcpp::Node::make_shared("pub_rgbd_node");
    // set depth image format parameter "camera.depth.image_rect_raw.format" to "png"
    g_node->declare_parameter<std::string>("camera.depth.image_rect_raw.format", "png");
    // image_transport will publish the video that can be compressed
    image_transport::ImageTransport it(g_node);
    image_transport::Publisher pub_color = it.advertise("/camera/color/image_raw", 10);
    image_transport::Publisher pub_depth = it.advertise("/camera/depth/image_rect_raw", 10);

    cv::Mat color_cv, depth_cv;

    while (rclcpp::ok()) {
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera (must be called in a loop)
        std_msgs::msg::Header header;
        header.stamp = rclcpp::Clock().now();

        rs2::frame color_frame = data.get_color_frame();
        rs2::frame depth_frame = data.get_depth_frame();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        color_cv = cv::Mat(cv::Size(640, 480), CV_8UC3, (void *) color_frame.get_data(), cv::Mat::AUTO_STEP);
        depth_cv = cv::Mat(cv::Size(640, 480), CV_16UC1, (void *) depth_frame.get_data(), cv::Mat::AUTO_STEP);

        sensor_msgs::msg::Image::SharedPtr msg_color = cv_bridge::CvImage(header, "bgr8", color_cv).toImageMsg();
        sensor_msgs::msg::Image::SharedPtr msg_depth = cv_bridge::CvImage(header, "mono16", depth_cv).toImageMsg();
        pub_color.publish(msg_color);
        pub_depth.publish(msg_depth);

        rclcpp::spin_some(g_node);
    }
}
