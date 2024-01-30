#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

class ImageSubscriber : public rclcpp::Node
{
public:
    ImageSubscriber() 
    : Node("image_subscriber")
    {
        setupSocket(8080);
    }
    void spin() {
        while (rclcpp::ok()) {
            receiveImageThroughSocket();
        }
    }

private:
    void setupSocket(int port) {
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd < 0) {
            perror("socket creation failed");
            exit(EXIT_FAILURE);
        }

        memset(&servaddr, 0, sizeof(servaddr));

        servaddr.sin_family = AF_INET;
        servaddr.sin_addr.s_addr = INADDR_ANY;
        servaddr.sin_port = htons(port);

        if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
            perror("bind failed");
            exit(EXIT_FAILURE);
        }
    }

    void receiveImageThroughSocket() {
        char buffer[MAX_BUFFER_SIZE];
        socklen_t len = sizeof(cliaddr);
        int n = recvfrom(sockfd, buffer, MAX_BUFFER_SIZE, MSG_WAITALL, (struct sockaddr *) &cliaddr, &len);
        if (n <= 0) {
            return; // Handle error or no data received
        }

        std::vector<uchar> buf(buffer, buffer + n);
        cv::Mat frame = cv::imdecode(buf, cv::IMREAD_COLOR);

        cv::imshow("Received Image", frame);
        cv::waitKey(1);
    }

    int sockfd;
    struct sockaddr_in servaddr, cliaddr;
    const int MAX_BUFFER_SIZE = 5 * 1024 * 1024;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto image_subscriber = std::make_shared<ImageSubscriber>();
    image_subscriber->spin();
    rclcpp::shutdown();
    return 0;
}
