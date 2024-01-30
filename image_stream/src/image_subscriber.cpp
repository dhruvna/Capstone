#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

const int MAX_BUFFER_SIZE = 5 * 1024 * 1024;

class ImageSubscriber : public rclcpp::Node
{
public:
    ImageSubscriber() 
    : Node("image_subscriber")
    {
        setupSocket("172.24.142.174", 8080);
    }

    ~ImageSubscriber() {
        close(sockfd);
    }
    void receiveAndDisplayImage() {
        while (rclcpp::ok()) {
            char buffer[1024] = {0}; // Increase buffer size if needed
            int bytes_received = recv(sockfd, buffer, 1024, 0);
            if (bytes_received < 0) {
                perror("recv failed");
                rclcpp::shutdown();
                break;
            } else if (bytes_received > 0) {
                RCLCPP_INFO(this->get_logger(), "Received message: %s", buffer);
            }
        }
    }

    void receiveImageThroughSocket() {
        while (rclcpp::ok()) {
            std::vector<uchar> buf(MAX_BUFFER_SIZE);
            int bytes_received = recv(sockfd, buf.data(), MAX_BUFFER_SIZE, 0);
            if (bytes_received < 0) {
                perror("recv failed");
                rclcpp::shutdown();
                break;
            }
            // Check if any data is received
            if (bytes_received > 0) {
                cv::Mat frame = cv::imdecode(buf, cv::IMREAD_COLOR);
                if (!frame.empty()) {
                    cv::imshow("Received Image", frame);
                    cv::waitKey(1);
                }
            }
        }
    }

private:
    void setupSocket(const char* ip, int port) {
        sockfd = socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd < 0) {
            perror("socket creation failed");
            rclcpp::shutdown();
            exit(EXIT_FAILURE);
        }

        servaddr.sin_family = AF_INET;
        servaddr.sin_port = htons(port);

        if(inet_pton(AF_INET, ip, &servaddr.sin_addr) <= 0) {
            perror("Invalid address/ Address not supported");
            rclcpp::shutdown();
            exit(EXIT_FAILURE);
        }

        if (connect(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
            perror("Connection Failed");
            rclcpp::shutdown();
            exit(EXIT_FAILURE);
        }
        RCLCPP_INFO(this->get_logger(), "Connected to server on Port %d", port);
    }

    int sockfd;
    struct sockaddr_in servaddr, cliaddr;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto image_subscriber = std::make_shared<ImageSubscriber>();
    image_subscriber->receiveAndDisplayImage();
    image_subscriber->receiveImageThroughSocket();
    rclcpp::shutdown();
    return 0;
}
