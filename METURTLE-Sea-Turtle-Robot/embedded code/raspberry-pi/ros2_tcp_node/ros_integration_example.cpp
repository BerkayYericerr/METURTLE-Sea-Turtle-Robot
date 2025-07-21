#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <vector>
#include <thread>
#include <mutex>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/epoll.h>
#include <fcntl.h>
#include <rclcpp/rclcpp.hpp>
#include <cstdint>
#include <cstring> // For memcpy
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/string.hpp>
#include <lifecycle_msgs/msg/state.hpp>  

#include "DefaSense310DefaultOutStruct.h"

#define MAX_EVENTS 10
#define PORT 1024
#define ADDRESS "192.168.2.200"

// Utility: Read config.ini
std::map<std::string, double> readConfig(const std::string& filename) {
    std::map<std::string, double> config;
    std::ifstream file(filename);
    std::string line;
    if (!file.is_open()) {
        std::cerr << "Error: Could not open config file!\n";
        return config;
    }
    while (std::getline(file, line)) {
        std::istringstream is_line(line);
        std::string key;
        double value;
        if (std::getline(is_line, key, '=') && (is_line >> value)) {
            key.erase(key.find_last_not_of(" \t") + 1);  // Trim spaces
            config[key] = value;
        }
    }
    return config;
}

// Utility: Make socket non-blocking
void makeSocketNonBlocking(int sockfd) {
    int flags = fcntl(sockfd, F_GETFL, 0);
    fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);
}

// Abstract EventHandler for Reactor
class EventHandler {
public:
    virtual void handleEvent(int fd, uint32_t events) = 0;
    virtual ~EventHandler() = default;
};

// Managed ROS 2 Lifecycle Node
class ManagedRosPublisher : public rclcpp_lifecycle::LifecycleNode {
private:
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr publisher_;
    std::mutex data_mutex_;
    S_DEFAULT_OUT latest_data_;
    double write_hz_;
    bool running_;

public:
    explicit ManagedRosPublisher(const std::string &node_name, double write_hz)
        : rclcpp_lifecycle::LifecycleNode(node_name), write_hz_(write_hz) {}

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &) override {
        publisher_ = this->create_publisher<std_msgs::msg::String>("tcp_data", 10);
        RCLCPP_INFO(get_logger(), "Node configured.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &) override {
        publisher_->on_activate();
        RCLCPP_INFO(get_logger(), "Node activated.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &) override {
        publisher_->on_deactivate();
        RCLCPP_INFO(get_logger(), "Node deactivated.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    void storeData(const S_DEFAULT_OUT &data) {
        std::lock_guard<std::mutex> lock(data_mutex_);

        latest_data_ = data; //Hold data

        printf("Data copied!\n");
    }

    std::string serializeData(const S_DEFAULT_OUT &data) {
        std::ostringstream oss;
        
        // Converting fields to a readable string format
        oss << "data_length: " << std::dec << data.data_length << ", ";
        oss << "imu_counter: " << std::dec << data.imu_counter << ", ";

        oss << "UTC_valid: " << std::dec << data.gnss_status.bits.UTC_valid << ", ";
        oss << "GNSS_heading_valid: " << std::dec << data.gnss_status.bits.GNSS_heading_valid << ", ";
        oss << "GNSS_fixok: " << std::dec << data.gnss_status.bits.GNSS_fixok << ", ";
        oss << "Number_of_Satel: " << std::dec << data.gnss_status.bits.Number_of_Satel << ", ";

        oss << "year: " << std::dec << data.gnss_utc.fields.year << ", ";
        oss << "month: " << std::dec << data.gnss_utc.fields.month << ", ";
        oss << "day: " << std::dec << data.gnss_utc.fields.day << ", ";
        oss << "hour: " << std::dec << data.gnss_utc.fields.hour << ", ";
        oss << "min: " << std::dec << data.gnss_utc.fields.min << ", ";
        oss << "sec: " << std::dec << data.gnss_utc.fields.sec << ", ";

        oss << "heading_valid: " << std::dec << data.ins_status.heading_valid << ", ";
        oss << "roll_pitch_valid: " << std::dec << data.ins_status.roll_pitch_valid << ", ";
        oss << "FOM1: " << std::dec << data.ins_status.FOM1 << ", ";

        oss << "ins_ypr: " << std::dec << data.ins_ypr << ", ";
        oss << "ins_quaternion: " << std::dec << data.ins_quaternion << ", ";
        oss << "ins_pos_llh: " << std::dec << data.ins_pos_llh << ", ";
        oss << "ins_vel_ned_mps: " << std::dec << data.ins_vel_ned_mps << ", ";
        oss << "ins_pos_uncertainty_met: " << std::dec << data.ins_pos_uncertainty_met << ", ";
        oss << "ins_vel_uncertainty_mps: " << std::dec << data.ins_vel_uncertainty_mps << ", ";
        oss << "ins_ypr_uncertainty_deg: " << std::dec << data.ins_ypr_uncertainty_deg << ", ";


        return oss.str();
    }

    void publishLoop() {
        while (running_ && rclcpp::ok()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1000 / write_hz_))); // Sleeping at start so that at start it gives enaugh time for package to be captured   

            std::lock_guard<std::mutex> lock(data_mutex_);
            if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                auto msg = std_msgs::msg::String();
                msg.data = serializeData(latest_data_); // Serialize struct to string before publishing
                publisher_->publish(msg);
            }
        }
    }
};

// Reactor Class
class Reactor {
private:
    int epoll_fd_;
    std::map<int, EventHandler *> handlers_;

public:
    Reactor() {
        epoll_fd_ = epoll_create1(0);
        if (epoll_fd_ == -1) {
            perror("epoll_create1 failed");
            exit(EXIT_FAILURE);
        }
    }

    void addHandler(int fd, EventHandler *handler, uint32_t events) {
        epoll_event ev{};
        ev.events = events;
        ev.data.ptr = handler;
        if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, fd, &ev) == -1) {
            perror("epoll_ctl failed");
            exit(EXIT_FAILURE);
        }
        handlers_[fd] = handler;
    }

    void run() {
        epoll_event events[MAX_EVENTS];
        while (rclcpp::ok()) {
            int num_events = epoll_wait(epoll_fd_, events, MAX_EVENTS, -1);
            for (int i = 0; i < num_events; ++i) {
                EventHandler *handler = static_cast<EventHandler *>(events[i].data.ptr);
                handler->handleEvent(events[i].data.fd, events[i].events);
            }
        }
    }
};

// TCP Client Handler
class ClientHandler : public EventHandler {
private:
    int client_fd_;
    Reactor &reactor_;
    std::shared_ptr<ManagedRosPublisher> ros_publisher_;
    double read_hz_;

public:
    ClientHandler(Reactor &reactor, const std::string &server_ip, int server_port, 
                  std::shared_ptr<ManagedRosPublisher> ros_pub, double read_hz)
        : reactor_(reactor), ros_publisher_(ros_pub), read_hz_(read_hz) {

        //Create a TCP socket
        client_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (client_fd_ == -1) {
            perror("Socket creation failed");
            exit(EXIT_FAILURE);
        }

        //Configure server address
        sockaddr_in server_addr{};
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(server_port);
        if (inet_pton(AF_INET, server_ip.c_str(), &server_addr.sin_addr) <= 0) {
            perror("Invalid address / Address not supported");
            exit(EXIT_FAILURE);
        }

        //Connect to the server
        if (connect(client_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            perror("Connection failed");
            exit(EXIT_FAILURE);
        }

        std::cout << "Connected to server " << server_ip << ":" << server_port << std::endl;
        makeSocketNonBlocking(client_fd_);

        //Register this client in the reactor
        reactor_.addHandler(client_fd_, this, EPOLLIN);

        //Send the initial message
        const char* message = "$ASWRG,080,32,0,0,3,255*";
        if (send(client_fd_, message, strlen(message), 0) < 0) {
            perror("Send failed");
        } else {
            std::cout << "Message sent: " << message << std::endl;
        }
    }

    void handleEvent(int fd, uint32_t events) override {
        if (events & EPOLLIN) {  // 6️⃣ Handle incoming server response
            char buffer[1024];
            ssize_t bytes_read = read(fd, buffer, sizeof(buffer) - 1);
            if (bytes_read > 0) {
                buffer[bytes_read] = '\0'; // Null-terminate
                std::cout << "Received from server: " << buffer << std::endl;

                bool flag = false;
                
                if(buffer[0] == 35){ //Check packet start #
                    uint16_t value;
                    std::memcpy(&value, buffer+1, sizeof(uint16_t));
                    if(value == 113){ //Check payload package size
                        //unsigned short crc_length = value; This can be used for dynamic array.

                        unsigned char data[113];
                        std::memcpy(data, buffer + 3, value);  // Extract payload

                    
                        unsigned short crc_received;
                        std::memcpy(&crc_received, buffer + (3 + 113), sizeof(uint16_t)); //Extract crc

                        if(calculateCRC(data, 113) == crc_received){ //Check 
                            //Store the data in struct
                            S_DEFAULT_OUT data;
                            std::memcpy(&data, buffer, std::min(bytes_read, (ssize_t)sizeof(S_DEFAULT_OUT)));
                            ros_publisher_->storeData(data);

                            flag = true;
                        }else{
                            printf("CRC failed!\n");
                        }
                    }
                }

                if(flag == false){
                    printf("Packet discarded!\n");
                }


            } else {
                std::cerr << "Server closed connection.\n";
                close(fd);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1000 / read_hz_)));
        }
    }

    unsigned short calculateCRC(unsigned char data[], unsigned short length){
        unsigned int i;
        unsigned short crc = 0;
        for(i=0; i<length; i++){
            crc = (unsigned char)(crc >> 8) | (crc << 8);
            crc ^= data[i];
            crc ^= (unsigned char)(crc & 0xff) >> 4;
            crc ^= crc << 12;
            crc ^= (crc & 0x00ff) << 5;
        }
        return crc;
    }

    int getClientFD() { return client_fd_; }
};


// Main function
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto config = readConfig("config.ini");
    double read_hz = config["tcp_read_hz"];
    double write_hz = config["ros_publish_hz"];

    auto managed_node = std::make_shared<ManagedRosPublisher>("tcp_to_ros2", write_hz);
    managed_node->configure();
    managed_node->activate();

    Reactor reactor;
    ClientHandler client(reactor, ADDRESS, PORT, managed_node, read_hz);
    //ServerHandler server(ADDRESS, PORT, reactor, managed_node, read_hz);//Deleted
    reactor.addHandler(client.getClientFD(), &client, EPOLLIN);

    std::thread ros_thread([&]() {
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(managed_node->get_node_base_interface());
        executor.spin();
    });

    std::thread publish_thread(&ManagedRosPublisher::publishLoop, managed_node);

    std::thread reactor_thread([&]() { reactor.run(); });

    ros_thread.join();
    publish_thread.join();
    reactor_thread.join();

    rclcpp::shutdown();
    return 0;
}