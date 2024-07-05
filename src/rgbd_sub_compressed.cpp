#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui/highgui.hpp>

// change the header file's name according to the ROS2 version
#if defined(ROS2_DISTRO_FOXY)
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif

#include <sensor_msgs/image_encodings.hpp>
#include <rclcpp/logging.hpp>

void ColorCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    try {
        cv::imshow("/camera/color/image_raw", cv_bridge::toCvShare(msg, "bgr8")->image);
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "cv_bridge exception: %s", e.what());
    }
}

void DepthCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    try {
        cv::imshow("/camera/depth/image_rect_raw", cv_bridge::toCvShare(msg, "mono16")->image);
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("depth"), "cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto g_node = rclcpp::Node::make_shared("compressed_sub_rgbd_node");
    // set image_transport parameter to "compressed"
    g_node->declare_parameter<std::string>("image_transport", "compressed");

    image_transport::ImageTransport it(g_node);
    image_transport::TransportHints hints(g_node.get());
    image_transport::Subscriber sub_color =
            it.subscribe("/camera/color/image_raw", 10, ColorCallback, &hints);
    image_transport::Subscriber sub_depth =
            it.subscribe("/camera/depth/image_rect_raw", 10, DepthCallback, &hints);

    rclcpp::Rate rate(30.0);
    while (rclcpp::ok()) {
        rclcpp::spin_some(g_node);
        rate.sleep();
    }
}