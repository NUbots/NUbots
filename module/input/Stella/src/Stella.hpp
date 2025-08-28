#ifndef MODULE_INPUT_STELLA_HPP
#define MODULE_INPUT_STELLA_HPP

#include <chrono>
#include <memory>
#include <nuclear>
#include <yaml-cpp/yaml.h>
#include <thread>
#include <atomic>

// Stella VSLAM includes
#include <stella_vslam/camera/base.h>
#include <stella_vslam/camera/equirectangular.h>
#include <stella_vslam/camera/fisheye.h>
#include <stella_vslam/camera/perspective.h>
#include <stella_vslam/config.h>
#include <stella_vslam/system.h>

// Socket publisher include
#include "socket_publisher/publisher.h"

// OpenCV includes
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

// NUbots message includes
#include "message/input/Image.hpp"

namespace module::input {

    class Stella : public NUClear::Reactor {
    private:
        /// @brief Stella VSLAM system
        std::shared_ptr<stella_vslam::system> slam_system;

        /// @brief SLAM configuration
        std::shared_ptr<stella_vslam::config> slam_config;

        /// @brief Frame publisher for debug visualization
        std::shared_ptr<stella_vslam::publish::frame_publisher> frame_publisher;

        /// @brief Map publisher for map visualization
        std::shared_ptr<stella_vslam::publish::map_publisher> map_publisher;

        /// @brief Socket publisher for map visualization
        std::shared_ptr<socket_publisher::publisher> socket_publisher;

        std::thread socket_thread;
        std::atomic<bool> socket_thread_running{false};
        void start_socket_once();
        void stop_socket_and_publishers();

    public:
        /// @brief Called by the powerplant to build and setup the Stella reactor.
        explicit Stella(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::input

#endif  // MODULE_INPUT_STELLA_HPP
