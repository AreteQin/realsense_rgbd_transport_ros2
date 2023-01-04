//
// Created by qin on 1/3/23.
//

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
//#include <sensor_msgs/image_encodings.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto g_node = rclcpp::Node::make_shared("pub_cam_node");

    cv::VideoCapture capture(0); //read the video from camera

    //image_transport will publish the video that can be compressed
    image_transport::ImageTransport it(g_node);
    image_transport::Publisher pub_color = it.advertise("/color", 1);

    cv::Mat image;

    while (rclcpp::ok()) {
        capture >> image; //load
        if (image.empty()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "image is empty");
            rclcpp::shutdown();
        }
//        cv::namedWindow("/color");
//        cv::imshow("/color", image);
//        cv::waitKey(1);
        pub_color.publish(cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg());
        rclcpp::spin_some(g_node);
    }
}