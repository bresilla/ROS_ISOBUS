#include <cmath>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "ros_isobus/simplecan.hpp"

class Cantest : public rclcpp::Node {
    private:
        std::string node_name;
        std::string interface;
        rclcpp::TimerBase::SharedPtr timer;
        scan::CANInterface can;

    public:
        Cantest(std::string interface) : Node("cantest"){
            can.initialize(interface.c_str());
            timer = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&Cantest::callback, this));
        }

    private:
        void callback(){
            uint8_t data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
            can.send_j1939_frame(0xFFDC, 6, 158, data);
            RCLCPP_INFO(this->get_logger(), "MESSAGE SENT");
        }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Cantest>("vcan0");
    rclcpp::spin(node);
    rclcpp::shutdown();
}
