#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/videoio.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h> 

class ImagePublisher : public rclcpp::Node
{
public:
    ImagePublisher() 
    : Node("image_publisher"), 
    video_capture_(ament_index_cpp::get_package_share_directory("image_stream") + "/resource/testvideo.mp4")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("video_frame", 10);
        //get video fps
        double fps = video_capture_.get(cv::CAP_PROP_FPS);
        //log it
        RCLCPP_INFO(this->get_logger(), "Input Video: %fFPS", fps);
        //create a timer that will send video at the corresponding frame rate
        timer_ = this->create_wall_timer(
            std::chrono::nanoseconds(static_cast<int>(1000000000 / fps)),
            std::bind(&ImagePublisher::timer_callback, this));
        setupSocket(8080);
    }

private:
    void timer_callback()
    {
        cv::Mat frame;
        if (!video_capture_.read(frame)) {
            RCLCPP_INFO(this->get_logger(), "End of video stream");
            rclcpp::shutdown();
            return;
        }
        sendImageThroughSocket(frame);
    }

    void setupSocket(int port) {
        server_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd < 0) {
            perror("socket creation failed");
            exit(EXIT_FAILURE);
        }

        servaddr.sin_family = AF_INET;
        servaddr.sin_addr.s_addr = INADDR_ANY;
        servaddr.sin_port = htons(port);

        if (bind(server_fd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
            perror("bind failed");
            exit(EXIT_FAILURE);
        }

        if (listen(server_fd, 3) < 0) {
            perror("listen failed");
            exit(EXIT_FAILURE);
        }

        RCLCPP_INFO(this->get_logger(), "Server Listening on Port %d", port);
    }

    void sendImageThroughSocket(const cv::Mat &frame) {
        int client_fd = accept(server_fd, (struct sockaddr *)&servaddr, (socklen_t*)&addrlen);
        if (client_fd < 0) {
            perror("accept failed");
            exit(EXIT_FAILURE);
        }

        std::vector<uchar> buffer;
        cv::imencode(".jpg", frame, buffer);

        int bytes_sent = send(client_fd, buffer.data(), buffer.size(), 0);
        if (bytes_sent < 0) {
            perror("send failed");
        }

        close(client_fd); // Close the connection after sending
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture video_capture_;
    int server_fd;
    struct sockaddr_in servaddr, client;
    socklen_t client_len = sizeof(client);
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePublisher>());
    rclcpp::shutdown();
    return 0;
}
