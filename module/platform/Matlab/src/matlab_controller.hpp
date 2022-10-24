#ifndef MODULE_PLATFORM_MATLABCONTROLLER_HPP
#define MODULE_PLATFORM_MATLABCONTROLLER_HPP

/*
 *
 */
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
// Networking Headers
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
// // Webots Headers
// #include <webots/Accelerometer.hpp>
// #include <webots/Camera.hpp>
// #include <webots/Device.hpp>
// #include <webots/Gyro.hpp>
// #include <webots/Motor.hpp>
// #include <webots/Node.hpp>
// #include <webots/PositionSensor.hpp>
// #include <webots/Robot.hpp>
// #include <webots/TouchSensor.hpp>

class MatlabServer {
public:
    MatlabServer(const int& timestep_, const uint16_t& server_port_) : timestep(timestep_), server_port(server_port_) {}
    // ~MatlabServer() override {
    //     // Shutdown Server && Socket
    // }

    MatlabServer(MatlabServer& other)            = delete;
    MatlabServer& operator=(MatlabServer& other) = delete;
    // Disable moving MatlabServer objects until we have tested that doing it doesn't break things
    MatlabServer(MatlabServer&& other)            = delete;
    MatlabServer& operator=(MatlabServer&& other) = delete;

    void run() {
        std::cout << "MatlabServer run" << std::endl;
        establish_server();
        // HACK
        while (true) {
            int addrlen = sizeof(address);
            int client_fd =
                accept(server_fd, reinterpret_cast<sockaddr*>(&address), reinterpret_cast<socklen_t*>(&addrlen));
            receive_joint_values(client_fd);
            // set_joint_values();
            close(client_fd);
        }
    }


private:
    int server_fd;
    const int timestep;
    struct sockaddr_in address;
    std::array<float, 18> joint_value;
    const uint16_t server_port;
    const std::array<std::string, 18> joint_name = {"right_ankle_roll",
                                                    "right_ankle_pitch",
                                                    "right_knee_pitch",
                                                    "right_hip_pitch",
                                                    "right_hip_roll [hip]",
                                                    "right_hip_yaw",
                                                    "left_hip_yaw",
                                                    "left_hip_roll [hip]",
                                                    "left_hip_pitch",
                                                    "left_knee_pitch",
                                                    "left_ankle_pitch",
                                                    "left_ankle_roll",
                                                    "right_shoulder_pitch [shoulder]",
                                                    "right_shoulder_roll",
                                                    "right_elbow_pitch",
                                                    "left_elbow_pitch",
                                                    "left_shoulder_roll",
                                                    "left_shoulder_pitch [shoulder]"};
    const std::array<std::string, 18> joint_numb = {"right_ankle_roll",
                                                    "right_ankle_pitch",
                                                    "right_knee_pitch",
                                                    "right_hip_pitch",
                                                    "right_hip_roll [hip]",
                                                    "right_hip_yaw",
                                                    "left_hip_yaw",
                                                    "left_hip_roll [hip]",
                                                    "left_hip_pitch",
                                                    "left_knee_pitch",
                                                    "left_ankle_pitch",
                                                    "left_ankle_roll",
                                                    "right_shoulder_pitch [shoulder]",
                                                    "right_shoulder_roll",
                                                    "right_elbow_pitch",
                                                    "left_elbow_pitch",
                                                    "left_shoulder_roll",
                                                    "left_shoulder_pitch [shoulder]"};

    void establish_server() {
        address.sin_family      = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port        = htons(server_port);
        int option              = 1;
        uint addrlen            = sizeof(address);

        server_fd = socket(AF_INET, SOCK_STREAM, 0);  // If 0, socket failed
        int svr_opt =
            setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &option, sizeof(option));  // If !0, failed
        int svr_bind   = bind(server_fd, reinterpret_cast<sockaddr*>(&address), addrlen);             // If < 0, failed
        int svr_listen = listen(server_fd, 1);                                                        // If < 0, failed

        std::cout << server_fd << std::endl;
        std::cout << svr_opt << std::endl;
        std::cout << svr_bind << std::endl;
        std::cout << svr_listen << std::endl;
        std::cout << inet_ntoa(address.sin_addr) << std::endl;

        if (server_fd == 0 || svr_opt != 0 || svr_bind < 0 || svr_listen < 0) {
            std::cout << "Establishing Server Failed..." << std::endl;
        }
        else {
            std::cout << "Server Established" << std::endl;
        }
    }

    void receive_joint_values(int client_file_descriptor) {
        if (client_file_descriptor > 0) {
            char buffer[256]  = {0};
            ssize_t valread   = read(client_file_descriptor, buffer, 256);
            char* buf_mem_add = buffer;
            int mem_offset    = 0;
            uint32_t iter     = 0;
            if (valread > 0) {
                while (sscanf(buf_mem_add, "%e,%n", &joint_value[iter], &mem_offset) == 1) {
                    buf_mem_add += mem_offset;
                    iter++;
                }
            }
            // DEBUG

            for (float jointVal : joint_value) {
                std::cout << jointVal << std::endl;
            }
        }
    }

    // void set_joint_values() {
    //     for (uint j = 0; j < joint_name.size(); j++) {
    //         webots::Motor* currentMotor = this->getMotor(joint_name[j]);
    //         currentMotor->setPosition(joint_value[j]);
    //     }
    //     printf("\nMotor positions set... \n");
    // }
};

// int main(int argc, char** argv) {

//     if (argc != 3) {
//         std::cerr << "Usage: " << argv[0] << " <TCP PORT> <CONTROLLER_TIME_STEP>" << std::endl;
//         return EXIT_FAILURE;
//     }

//     uint16_t port_number = 0;
//     port_number = static_cast<uint16_t> (std::stoi(argv[1]));
//     int time_step = std::stoi(argv[2]);

//     std::unique_ptr<MatlabServer> robot = std::make_unique<MatlabServer>(time_step, port_number);
//     robot->run();

//     return EXIT_SUCCESS;
// }


/*

            // while (robot->step(time_step) != -1) {
            //
            //     currentMotor->setPosition(1.5707);
            //
            //     for(const auto &motor : joint) {
            //         webots::Motor* currentMotor = robot->getMotor(motor);
            //
            //     }
            //

            // }

*/

#endif  // MODULE_PLATFORM_MATLAB_HPP
