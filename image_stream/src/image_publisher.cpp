#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/videoio.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

class ImagePublisher : public rclcpp::Node
{
public:
    // ImagePublisher() : Node("image_publisher")
    // {
    //     std::string package_share_directory = ament_index_cpp::get_package_share_directory("image_stream");
    //     image_path_ = package_share_directory + "/resource/boat.png";

    //     publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_topic", 10);
    //     timer_ = this->create_wall_timer(
    //         std::chrono::milliseconds(1000),
    //         std::bind(&ImagePublisher::timer_callback, this));
    // }
    ImagePublisher() : Node("image_publisher"), cap_(0)
    {
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open video stream");
            throw std::runtime_error("Could not open video stream");
        }
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("video_frame", 10);
        // Assuming a video with 30 FPS
        double fps = cap_.get(cv::CAP_PROP_FPS);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000 / fps)),
            std::bind(&ImagePublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        cv::Mat frame;
        if (!cap_.read(frame)) {  // Capture a frame from the video source
            RCLCPP_ERROR(this->get_logger(), "Failed to capture frame");
            return;
        }
        // Convert to a ROS2 sensor_msgs::msg::Image and publish
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

        // cv::Mat image = cv::imread(image_path_, cv::IMREAD_COLOR);
        // if (image.empty()) {
        //     RCLCPP_ERROR(this->get_logger(), "Failed to load image 'resource/boat.png'");
        //     return;
        // }

        // auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
        publisher_->publish(*msg);
        //print success to console
        // RCLCPP_INFO(this->get_logger(), "Published image: 'resource/boat.png'");
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;  // Video capture object

    // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    // rclcpp::TimerBase::SharedPtr timer_;
    // std::string image_path_; 
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImagePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
