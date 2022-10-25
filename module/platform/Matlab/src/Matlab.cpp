/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2021 NUbots <nubots@nubots.net>
 */

#include "Matlab.hpp"

#include <chrono>
#include <fmt/format.h>
#include <iostream>
#include <string>

#include "clock/clock.hpp"
#include "matlab_controller.hpp"

#include "extension/Configuration.hpp"

#include "message/behaviour/ServoCommand.hpp"

#include "utility/behaviour/Action.hpp"
#include "utility/input/LimbID.hpp"


namespace module::platform {

    using extension::Configuration;

    using message::behaviour::ServoCommands;

    using utility::behaviour::ActionPriorities;
    using utility::behaviour::RegisterAction;
    using utility::input::LimbID;
    using utility::input::ServoID;


    Matlab::Matlab(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), subsumption_id(size_t(this) * size_t(this) - size_t(this)) {
        // do a little configurating
        on<Configuration>("Matlab.yaml").then([this](const Configuration& config) {
            log_level      = config["log_level"].as<NUClear::LogLevel>();
            cfg.tcp_port   = config["tcp_port"];
            cfg.servo_gain = config["servo_gain"];


            // Configure the server
            address.sin_family      = AF_INET;
            address.sin_addr.s_addr = INADDR_ANY;
            address.sin_port        = htons(cfg.tcp_port);
            int option              = 1;
            uint addrlen            = sizeof(address);

            server_fd      = socket(AF_INET, SOCK_STREAM, 0);  // If 0, socket failed
            int svr_opt    = setsockopt(server_fd,
                                     SOL_SOCKET,
                                     SO_REUSEADDR | SO_REUSEPORT,
                                     &option,
                                     sizeof(option));                                       // If !0, failed
            int svr_bind   = bind(server_fd, reinterpret_cast<sockaddr*>(&address), addrlen);  // If < 0, failed
            int svr_listen = listen(server_fd, 1);                                             // If < 0, failed

            if (server_fd == 0 || svr_opt != 0 || svr_bind < 0 || svr_listen < 0) {
                log<NUClear::WARN>("Establishing Server Failed...");
                log<NUClear::DEBUG>("server_fd: ", server_fd);
                log<NUClear::DEBUG>("svr_opt: ", svr_opt);
                log<NUClear::DEBUG>("svr_bind: ", svr_bind);
                log<NUClear::DEBUG>("svr_listen: ", svr_listen);
                log<NUClear::DEBUG>("sin_addr: ", inet_ntoa(address.sin_addr));
            }
            else {
                log<NUClear::INFO>("Establishing Server Succeeded.");
            }
        });

        emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(
            RegisterAction{subsumption_id,
                           "Matlab",
                           {
                               // Limb sets required by the walk engine:
                               std::pair<double, std::set<LimbID>>(
                                   0,
                                   {LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM}),
                           },
                           [this](const std::set<LimbID>& given_limbs) {
                               // nothing
                           },
                           [this](const std::set<LimbID>& taken_limbs) {
                               // nothing
                           },
                           [](const std::set<ServoID>& /*unused*/) {
                               // nothing
                           }}));


        // Every timestep, we want to send the current joint values to the robot
        on<Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>>().then([this] {
            // Create servo_commands message
            auto servo_commands = std::make_unique<ServoCommands>();
            servo_commands->commands.reserve(n_servos);
            const NUClear::clock::time_point time = NUClear::clock::now();

            int addrlen   = sizeof(address);
            int client_fd = -1;
            // client_fd =
            //     accept(server_fd, reinterpret_cast<sockaddr*>(&address), reinterpret_cast<socklen_t*>(&addrlen));

            if (client_fd > 0) {
                std::array<float, n_servos> joint_values;
                char buffer[256]  = {0};
                ssize_t valread   = read(client_fd, buffer, 256);
                char* buf_mem_add = buffer;
                int mem_offset    = 0;
                uint32_t iter     = 0;
                // Read the joint values from the matlab server
                if (valread > 0) {
                    while (sscanf(buf_mem_add, "%e,%n", &joint_values[iter], &mem_offset) == 1) {
                        buf_mem_add += mem_offset;
                        iter++;
                    }
                }

                // Add the joint values to the servo_commands message
                for (int i = 0; i < n_servos; i++) {
                    servo_commands->commands
                        .emplace_back(subsumption_id, time, servo_ids[i], joint_values[i], cfg.servo_gain, 100);
                    log<NUClear::DEBUG>("SERVO ID ", servo_ids[i], ", Value: ", joint_values[i]);
                }

                // Emit servo commands them to system
                emit(std::move(servo_commands));

                // Update the priority in the subsumption system
                update_priority(cfg.matlab_priority);

                // Close the client socket
                close(client_fd);
            }

            // ****** TESTING: Send random servo values ***** //
            for (uint32_t i = 0; i < n_servos; i++) {
                float random   = ((float) rand()) / (float) RAND_MAX;
                float min      = -3.14;
                float max      = 3.14;
                float range    = max - min;
                float rand_num = (random * range) + min;
                servo_commands->commands.emplace_back(subsumption_id, time, i, rand_num, cfg.servo_gain, 100);
            }

            // Emit them to system
            emit(std::move(servo_commands));

            // Update the priority
            update_priority(cfg.matlab_priority);

            // Close the client socket
            close(client_fd);
        });
    }

    void Matlab::update_priority(const float& priority) {
        emit(std::make_unique<ActionPriorities>(ActionPriorities{subsumption_id, {priority}}));
    }
}  // namespace module::platform
