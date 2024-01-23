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
    ImagePublisher() 
    : Node("image_publisher"), 
    video_capture_(ament_index_cpp::get_package_share_directory("image_stream") + "/resource/testvideo.mp4")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("video_frame", 10);
        // Assuming a video with 30 FPS
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33), // Adjust this value to the frame rate of your video
            std::bind(&ImagePublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        cv::Mat frame;
        if (!video_capture_.read(frame)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to capture frame");
            rclcpp::shutdown(); // Optionally shut down if the video is over
            return;
        }
        // Convert to a ROS2 sensor_msgs::msg::Image and publish
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        publisher_->publish(*msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture video_capture_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePublisher>());
    rclcpp::shutdown();
    return 0;
}
