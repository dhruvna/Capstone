#include <memory>
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <arpa/inet.h>

#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <cstring>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"

class ImageSubscriber : public rclcpp::Node
{
public:
    ImageSubscriber() : Node("image_subscriber"), sockfd(-1)
    {
        std::string publisher_public_ip = "192.168.0.2";
        int publisher_port = 9001;

        this->declare_parameter<std::string>("publisher_ip", publisher_public_ip);
        this->declare_parameter<int>("publisher_port", publisher_port);

        this->get_parameter("publisher_ip", publisher_ip);
        this->get_parameter("publisher_port", publisher_port);

        RCLCPP_INFO(this->get_logger(), "Connecting to Publisher IP: %s, Port: %d", publisher_ip.c_str(), publisher_port);

        init_udp_socket();
    }
    ~ImageSubscriber() 
    {
        if(sockfd != -1)
        {
            close(sockfd);
        }
    }

private:
    void init_udp_socket() 
    {
        if((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Socket creation failed.");
            exit(EXIT_FAILURE);
        }
        
        memset(&servaddr, 0, sizeof(servaddr));
        servaddr.sin_family = AF_INET;
        servaddr.sin_port = htons(publisher_port);
        servaddr.sin_addr.s_addr = inet_addr(publisher_ip.c_str());
        // servaddr.sin_addr.s_addr = htonl(INADDR_ANY);

        if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Bind failed");
            exit(EXIT_FAILURE);
        }
        start_receiving2();
    }

    void start_receiving2() {
        char buffer[1024]; // Buffer for the incoming message
        struct sockaddr_in cliaddr;
        unsigned int len = sizeof(cliaddr);

        while (rclcpp::ok()) {
            int n = recvfrom(sockfd, buffer, sizeof(buffer) - 1, 0, (struct sockaddr *)&cliaddr, &len);
            if (n > 0) {
                buffer[n] = '\0'; // Null-terminate the received string
                RCLCPP_INFO(this->get_logger(), "Received message: %s", buffer);
            }
        }
    }

    void start_receiving() 
    {   
        char buffer[65507];
        struct sockaddr_in cliaddr;
        unsigned int len = sizeof(cliaddr);

        while (rclcpp::ok()) 
        {
            int n = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr *)&cliaddr, &len);
            buffer[n] = '\0';

            std::string received_msg(buffer);
            // RCLCPP_INFO(this->get_logger(), "Received message: %s", received_msg.c_str());

            if(received_msg == "END OF STREAM") {
                RCLCPP_INFO(this->get_logger(), "End of stream detected.");
                break;
            } else {
                std::vector<uchar> data(buffer, buffer+n);
                cv::Mat frame = cv::imdecode(cv::Mat(data), cv::IMREAD_COLOR);
                
                if(!frame.empty()) 
                {
                    cv::imshow("Received Frame", frame);
                    cv::waitKey(1);
                }
            }
        }
    }

    int sockfd;
    struct sockaddr_in servaddr;
    std::string publisher_ip;
    int publisher_port;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto image_subscriber = std::make_shared<ImageSubscriber>();
    rclcpp::shutdown();
    return 0;
}
