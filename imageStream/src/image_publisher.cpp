#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class ImagePublisher : public rclcpp::Node
{
public:
    ImagePublisher() : Node("image_publisher")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_topic", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ImagePublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        cv::Mat image = cv::imread("resource/boat.png", cv::IMREAD_COLOR);
        if (image.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load image 'resource/boat.png'");
            return;
        }

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
        publisher_->publish(*msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImagePublisher>();
    rclcpp::spin(std::make_shared<ImagePublisher>());
    rclcpp::shutdown();
    return 0;
}