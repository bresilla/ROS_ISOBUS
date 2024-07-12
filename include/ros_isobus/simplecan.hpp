#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

namespace scan{
    class CANInterface { 
        private:
            int soc;
            struct sockaddr_can addr;
            struct ifreq ifr;
            bool initialized;

        public:
            // Default constructor
            CANInterface() : soc(-1), initialized(false) {}
            ~CANInterface() { if (initialized) {close(soc); } }

            CANInterface(const char* interfaceName) : soc(-1), initialized(false) {
                initialize(interfaceName);
            }

            bool isInitialized() const { return initialized; }

            // Method to initialize the interface
            bool initialize(const char* interfaceName) {
                if (initialized)
                    return true;  // Already initialized
                soc = socket(PF_CAN, SOCK_RAW, CAN_RAW);
                if (soc < 0) {
                    perror("Socket creation failed");
                    return false;
                }
                std::strcpy(ifr.ifr_name, interfaceName);
                ioctl(soc, SIOCGIFINDEX, &ifr);
                std::memset(&addr, 0, sizeof(addr));
                addr.can_family = AF_CAN;
                addr.can_ifindex = ifr.ifr_ifindex;
                if (bind(soc, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
                    perror("Socket bind failed");
                    close(soc);
                    return false;
                }
                initialized = true;
                return true;
            }

            // Method to send a CAN frame
            bool send_frame(uint32_t id, unsigned char *data, unsigned int dlc) {
                if (!initialized) {
                    std::cerr << "Interface not initialized" << std::endl;
                    return false;
                }
                if (dlc > CAN_MAX_DLEN) {
                    std::cerr << "Data length code too large" << std::endl;
                    return false;
                }
                struct can_frame frame;
                frame.can_id = id | CAN_EFF_FLAG; // Set the Extended Frame Format flag
                frame.can_dlc = dlc;
                std::memcpy(frame.data, data, dlc);
                int bytes_sent = write(soc, &frame, sizeof(frame));
                if (bytes_sent != sizeof(frame)) {
                    perror("Write to socket failed");
                    return false;
                }
                return true;
            }

                        
            // Method to receive a CAN frame
            bool receive_frame(uint32_t& id, unsigned char* data, unsigned int& dlc) {
                if (!initialized) {
                    std::cerr << "Interface not initialized" << std::endl;
                    return false;
                }
                struct can_frame frame;
                int bytes_received = read(soc, &frame, sizeof(frame));
                if (bytes_received < 0) {
                    perror("Read from socket failed");
                    return false;
                }
                if (bytes_received < sizeof(frame)) {
                    std::cerr << "Incomplete frame received" << std::endl;
                    return false;
                }
                id = static_cast<uint32_t>(frame.can_id & CAN_EFF_MASK);
                dlc = frame.can_dlc;
                std::memcpy(data, frame.data, dlc);
                return true;
            }

            // Method to send a J1939 frame
            bool send_j1939_frame(uint32_t pgn, uint8_t priority, uint8_t source, unsigned char data[8]) {
                if (!initialized) {
                    std::cerr << "Interface not initialized" << std::endl;
                    return false;
                }

                if (sizeof(data) > 8) {
                    std::cerr << "Data size too large, must be 64 bits (8 bytes) or less" << std::endl;
                    return false;
                }

                uint32_t id = ((priority & 0x7) << 26) | ((pgn & 0x3FFFF) << 8) | (source & 0xFF);
                
                struct can_frame frame;
                frame.can_id = id | CAN_EFF_FLAG; // Set the Extended Frame Format flag
                frame.can_dlc = 8; // For J1939, DLC is always 8
                std::memcpy(frame.data, data, 8);
                
                int bytes_sent = write(soc, &frame, sizeof(frame));
                if (bytes_sent != sizeof(frame)) {
                    perror("Write to socket failed");
                    return false;
                }
                return true;
            }

            // Method to receive a J1939 frame
            bool receive_j1939_frame(uint8_t& priority, uint32_t& pgn, uint8_t& source, unsigned char data[8]) {
                if (!initialized) {
                    std::cerr << "Interface not initialized" << std::endl;
                    return false;
                }

                struct can_frame frame;
                int bytes_received = read(soc, &frame, sizeof(frame));
                if (bytes_received < 0) {
                    perror("Read from socket failed");
                    return false;
                }
                if (bytes_received < sizeof(frame)) {
                    std::cerr << "Incomplete frame received" << std::endl;
                    return false;
                }

                // Extract J1939 components from the CAN ID
                uint32_t id = frame.can_id & CAN_EFF_MASK;
                priority = (id >> 26) & 0x7;
                pgn = (id >> 8) & 0x3FFFF;
                source = id & 0xFF;

                // Copy the data from the CAN frame
                std::memcpy(data, frame.data, frame.can_dlc);

                return true;
            }
            // Method to receive a J1939 frame
            bool receive_j1939_and_raw_frame(uint32_t& id, uint8_t& priority, uint32_t& pgn, uint8_t& source, unsigned char data[8]) {
                if (!initialized) {
                    std::cerr << "Interface not initialized" << std::endl;
                    return false;
                }

                struct can_frame frame;
                int bytes_received = read(soc, &frame, sizeof(frame));
                if (bytes_received < 0) {
                    perror("Read from socket failed");
                    return false;
                }
                if (bytes_received < sizeof(frame)) {
                    std::cerr << "Incomplete frame received" << std::endl;
                    return false;
                }

                // Extract J1939 components from the CAN ID
                id = frame.can_id & CAN_EFF_MASK;
                priority = (id >> 26) & 0x7;
                pgn = (id >> 8) & 0x3FFFF;
                source = id & 0xFF;

                // Copy the data from the CAN frame
                std::memcpy(data, frame.data, frame.can_dlc);

                return true;
            }
    };
}
