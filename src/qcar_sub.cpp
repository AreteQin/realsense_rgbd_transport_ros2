//
// Created by qin on 2/27/24.
//
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

// change the header file's name according to the ROS2 version
#if defined(ROS2_DISTRO_FOXY)
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif

#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

class ImageSubscriber : public rclcpp::Node {
public:
    ImageSubscriber(const std::string& topicName, const std::string& imageType)
            : Node("image_subscriber"), startTime_(std::chrono::high_resolution_clock::now()) {
        // Custom QoS profile
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

        RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", topicName.c_str());

        if (imageType == "CompressedImage") {
            subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
                    topicName, qos, std::bind(&ImageSubscriber::compressedImageCallback, this, _1));
        } else if (imageType == "Image") {
            subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
                    topicName, qos, std::bind(&ImageSubscriber::imageCallback, this, _1));
        } else {
            RCLCPP_ERROR(this->get_logger(), "Incorrect topic type!");
        }
    }

private:
    void compressedImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        auto currentImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        imageDisplay(currentImage);
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        auto currentImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        imageDisplay(currentImage);
    }

    void imageDisplay(cv::Mat& currentFrame) {
        auto now = std::chrono::high_resolution_clock::now();
        auto timeDelta = std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime_).count();
        startTime_ = now;

        std::string fps = std::to_string(1000.0 / timeDelta) + " FPS";
        std::string imageInfo = std::to_string(currentFrame.cols) + "x" + std::to_string(currentFrame.rows);

        cv::putText(currentFrame, fps + " " + imageInfo, cv::Point(50, 50), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 0, 0), 2);
        cv::imshow("Camera Video Stream", currentFrame);
        cv::waitKey(1);
    }

    rclcpp::SubscriptionBase::SharedPtr subscription_;
    std::chrono::high_resolution_clock::time_point startTime_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    std::string topicName;
    std::cout << "Please input the name of the image node you will like to subscribe to (example: /qcar/csi_left): ";
    std::cin >> topicName;

    std::string imageType;
    std::cout << "Please input image message type.\nFor compressed image topic use: CompressedImage, for regular Image topic use: Image" << std::endl;
    std::cin >> imageType;

    auto imageSubscriber = std::make_shared<ImageSubscriber>(topicName, imageType);

    rclcpp::spin(imageSubscriber);
    rclcpp::shutdown();
    return 0;
}
