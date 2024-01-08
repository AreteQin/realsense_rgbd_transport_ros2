//#include <memory>
//
//#include "rclcpp/rclcpp.hpp"
//#include "std_msgs/msg/string.hpp"
//using std::placeholders::_1;
//
//class MinimalSubscriber : public rclcpp::Node
//{
//public:
//  MinimalSubscriber()
//  : Node("minimal_subscriber")
//  {
//    subscription_ = this->create_subscription<std_msgs::msg::String>(
//      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
//  }
//
//private:
//  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
//  {
//    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
//  }
//  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
//};
//
//int main(int argc, char * argv[])
//{
//  rclcpp::init(argc, argv);
//  rclcpp::spin(std::make_shared<MinimalSubscriber>());
//  rclcpp::shutdown();
//  return 0;
//}

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <rclcpp/logging.hpp>

void ColorCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    try {
        cv::imshow("D435/color", cv_bridge::toCvShare(msg, "bgr8")->image);
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "cv_bridge exception: %s", e.what());
    }
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
    auto g_node = rclcpp::Node::make_shared("sub_cam_node");

    cv::namedWindow("D435/color");
    cv::namedWindow("D435/depth");
    image_transport::ImageTransport it(g_node);
    image_transport::Subscriber sub_color = it.subscribe("D435/color", 1, ColorCallback);
    image_transport::Subscriber sub_depth = it.subscribe("D435/depth", 1, DepthCallback);
    rclcpp::Rate rate(30.0);
    while (rclcpp::ok()) {
        rclcpp::spin_some(g_node);
        rate.sleep();
    }
}