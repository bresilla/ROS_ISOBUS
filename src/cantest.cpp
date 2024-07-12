#include <cmath>
#include <string>
#include "isobus/isobus/can_NAME.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_isobus/simplecan.hpp"

#include "isobus/isobus/can_network_manager.hpp"
#include "isobus/hardware_integration/socket_can_interface.hpp"
#include "isobus/hardware_integration/can_hardware_interface.hpp"
#include "isobus/isobus/can_partnered_control_function.hpp"

class Cantest : public rclcpp::Node {
    private:
        std::string node_name;
        std::string interface;
        rclcpp::TimerBase::SharedPtr timer;
        scan::CANInterface can;
        std::shared_ptr<isobus::SocketCANInterface> can_driver;
        isobus::NAME isoname;
        std::shared_ptr<isobus::InternalControlFunction> ecu;

    public:
        Cantest(std::string interface) : Node("cantest"){
            can.initialize(interface.c_str());
            timer = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&Cantest::callback, this));
            can_driver = std::make_shared<isobus::SocketCANInterface>(interface.c_str());
            isobus::CANHardwareInterface::set_number_of_can_channels(1);
            isobus::CANHardwareInterface::assign_can_channel_frame_handler(0, can_driver);

            isobus::CANNetworkManager::CANNetwork.add_global_parameter_group_number_callback(0xFFDC, [](const isobus::CANMessage &CANMessage, void *){
                std::cout << CANMessage.get_data_length() << std::endl;
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MESSAGE RECEIVED");
            }, nullptr);

            if(isobus::CANHardwareInterface::start() || !can_driver->get_is_valid()){
                RCLCPP_ERROR(this->get_logger(), "Failed to start CAN hardware interface");
            }

            isoname.set_arbitrary_address_capable(true);
            isoname.set_industry_group(1);
            isoname.set_device_class(0);
            isoname.set_function_code(static_cast<std::uint8_t>(isobus::NAME::Function::SteeringControl));
            isoname.set_identity_number(2);
            isoname.set_ecu_instance(0);
            isoname.set_function_instance(0);
            isoname.set_device_class_instance(0);
            isoname.set_manufacturer_code(1407);

            ecu = isobus::CANNetworkManager::CANNetwork.create_internal_control_function(isoname, 0);
        }

    private:
        void propa_callback(const isobus::CANMessage &CANMessage, void *){
          std::cout << CANMessage.get_data_length() << std::endl;
        }
        void callback(){
            std::array<std::uint8_t, isobus::CAN_DATA_LENGTH> messageData = {1};
            isobus::CANNetworkManager::CANNetwork.send_can_message(0xEF00, messageData.data(), isobus::CAN_DATA_LENGTH, ecu);

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
