#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <rclcpp/logging.hpp>
#include <compressed_image_transport/compressed_subscriber.h>
#include <sensor_msgs/msg/compressed_image.hpp>

void ColorCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    try {
        cv::imshow("D435/color", cv_bridge::toCvShare(msg, "bgr8")->image);
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "cv_bridge exception: %s", e.what());
    }
    // calculate time delay
    auto now = rclcpp::Clock().now();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "time now: %f", now.seconds());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "time sent: %f", msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);
}

void DepthCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    try {
        cv::imshow("D435/depth", cv_bridge::toCvShare(msg, "mono16")->image);
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("depth"), "cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto g_node = rclcpp::Node::make_shared("compressed_sub_rgbd_node");
    // TransportHints does not actually declare the parameter
    g_node->declare_parameter<std::string>("image_transport", "compressed");
    // set depth image format parameter "D435.depth.format" to "png"
    g_node->declare_parameter<std::string>("D435.depth.compressed.format", "png");

    cv::namedWindow("D435/color");
    cv::namedWindow("D435/depth");

    image_transport::ImageTransport it(g_node);
    image_transport::TransportHints hints(g_node.get());
    image_transport::Subscriber sub_color = it.subscribe("/D435/color", 1, ColorCallback, &hints);
    image_transport::Subscriber sub_depth = it.subscribe("/D435/depth", 1, DepthCallback, &hints);

    rclcpp::Rate rate(30.0);
    while (rclcpp::ok()) {
        rclcpp::spin_some(g_node);
        rate.sleep();
    }
}