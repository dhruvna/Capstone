#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

class ImageSubscriber : public rclcpp::Node
{
public:
    ImageSubscriber() 
    : Node("image_subscriber")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "video_frame", 10,
            std::bind(&ImageSubscriber::image_callback, this, std::placeholders::_1));
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (msg->header.frame_id == "end_of_stream") {
            RCLCPP_INFO(this->get_logger(), "Video stream ended");
            rclcpp::shutdown();
            return;
        }
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cv::imshow("Video Frame", frame);
        cv::waitKey(1); // Use a small delay in waitKey for the display to be responsive
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSubscriber>());
    rclcpp::shutdown();
    return 0;
}
