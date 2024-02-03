//#include <chrono>
//#include <memory>
//#include <librealsense2/rs.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <sensor_msgs/msg/image.hpp>
//#include <rclcpp/rclcpp.hpp>
//
//using namespace std::chrono_literals;
//
//class MinimalPublisher : public rclcpp::Node {
//public:
//    MinimalPublisher()
//            : Node("minimal_publisher"), count_(0) {
//        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
//                "color_image", video_qos, colorCallback);
//    }
//
//private:
//    void colorCallback() {
//        auto message = sensor_msgs::msg::Image();
//        message.data = "Hello, world! " + std::to_string(count_++);
//        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
//        publisher_->publish(message);
//    }
//
//    rclcpp::TimerBase::SharedPtr timer_;
//    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
//    size_t count_;
//    rclcpp::QoS video_qos{10};
//};
//
//int main(int argc, char *argv[]) {
//    rclcpp::init(argc, argv);
//    rclcpp::spin(std::make_shared<MinimalPublisher>());
//    rclcpp::shutdown();
//    return 0;
//}

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <compressed_image_transport/compressed_publisher.h>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
//#include <sensor_msgs/image_encodings.hpp>
#include <librealsense2/rs.hpp> // Include Intel RealSense Cross Platform API

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // Declare the RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    //cfg.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    // Start streaming with the default recommended configuration
    pipe.start(cfg);

    auto g_node = rclcpp::Node::make_shared("sub_cam_node");
    // set depth image format parameter to png
    g_node->declare_parameter<std::string>("D435.depth.format", "png");
    //image_transport will publish the video that can be compressed
    image_transport::ImageTransport it(g_node);
    image_transport::Publisher pub_color = it.advertise("/D435/color", 1);
    image_transport::Publisher pub_depth = it.advertise("/D435/depth", 1);

//    cv::Mat image = cv::imread(argv[1], cv::IMREAD_COLOR);
//    std_msgs::msg::Header hdr;
//
//    rclcpp::WallRate loop_rate(5);
//    while (rclcpp::ok()) {
//        pub.publish(msg);
//        rclcpp::spin_some(node);
//        loop_rate.sleep();
//    }
//    auto pub_color = g_node->create_publisher<sensor_msgs::msg::CompressedImage>("D435/color", 1);
//    auto pub_depth = g_node->create_publisher<sensor_msgs::msg::CompressedImage>("D435/depth", 1);

    cv::Mat color_cv, depth_cv;
//    cv::namedWindow("D435/color");
//    cv::namedWindow("D435/depth");

    while (rclcpp::ok()) {
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera (must be called in a loop)
        std_msgs::msg::Header header;
        header.stamp = rclcpp::Clock().now();

//    rs2::frame color_frame = color_map.colorize(data.get_color_frame());
        rs2::frame color_frame = data.get_color_frame();
        rs2::frame depth_frame = data.get_depth_frame();
        // Query frame size (width and height)
//        const int w = depth_frame.as<rs2::video_frame>().get_width();
//        const int h = depth_frame.as<rs2::video_frame>().get_height();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        color_cv = cv::Mat(cv::Size(640, 480), CV_8UC3, (void *) color_frame.get_data(), cv::Mat::AUTO_STEP);
        depth_cv = cv::Mat(cv::Size(640, 480), CV_16UC1, (void *) depth_frame.get_data(), cv::Mat::AUTO_STEP);

//        cv::imshow("D435/color", color_cv);
//        cv::imshow("D435/depth", depth_cv);
//        cv::waitKey(1);

        // Print header and current time precisely until milliseconds
//        std::cout << "Header: " << header.stamp.sec << "." << header.stamp.nanosec << std::endl;
//        std::cout << "Current time: " << std::chrono::duration_cast<std::chrono::milliseconds>(
//                std::chrono::system_clock::now().time_since_epoch()).count() << std::endl;

//    depth_cv = cv::Mat(cv::Size(640, 480), CV_8UC3, (void *) depth_frame.get_data(), cv::Mat::AUTO_STEP);
//        pub_color.publish(cv_bridge::CvImage(header, "bgr8", color_cv).toImageMsg());
//        pub_depth.publish(cv_bridge::CvImage(header, "mono16", depth_cv).toImageMsg());
//        pub_color->publish(*cv_bridge::CvImage(header, "bgr8", color_cv).toCompressedImageMsg());
//        pub_depth->publish(*cv_bridge::CvImage(header, "mono16", depth_cv).toCompressedImageMsg());
        sensor_msgs::msg::Image::SharedPtr msg_color = cv_bridge::CvImage(header, "bgr8", color_cv).toImageMsg();
        sensor_msgs::msg::Image::SharedPtr msg_depth = cv_bridge::CvImage(header, "mono16", depth_cv).toImageMsg();
        pub_color.publish(msg_color);
        pub_depth.publish(msg_depth);

        rclcpp::spin_some(g_node);
    }
}