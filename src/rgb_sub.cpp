//
// Created by qin on 1/3/23.
//

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <rclcpp/logging.hpp>

void ColorCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    try {
        cv::imshow("/color", cv_bridge::toCvShare(msg, "bgr8")->image);
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto g_node = rclcpp::Node::make_shared("sub_cam_node");
    // TransportHints does not actually declare the parameter
    g_node->declare_parameter<std::string>("image_transport", "compressed");
    image_transport::TransportHints hints(g_node.get());

//    cv::namedWindow("/color");
    image_transport::ImageTransport it(g_node);
//    image_transport::Subscriber sub_color = it.subscribe("D435/color", 1, ColorCallback);
    image_transport::Subscriber sub_color = it.subscribe("/D435/color", 1, ColorCallback, &hints);
    rclcpp::Rate rate(30.0);
    while (rclcpp::ok()) {
        rclcpp::spin_some(g_node);
        rate.sleep();
    }
}