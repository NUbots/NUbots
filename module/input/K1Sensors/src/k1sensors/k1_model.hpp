#ifndef MODULE_INPUT_K1SENSORS_K1MODEL_HPP
#define MODULE_INPUT_K1SENSORS_K1MODEL_HPP

#include <Eigen/Geometry>
#include <memory>
#include <string>

// Forward declaration — avoids pulling tinyrobotics headers into every TU that
// includes K1Sensors.hpp
namespace message::input {
    class Sensors;
}  // namespace message::input

namespace module::input {

    /// @brief Opaque wrapper around the tinyrobotics K1 model.
    /// The implementation (and all heavy tinyrobotics template instantiations) live
    /// exclusively in k1_model.cpp so they do not bloat other translation units.
    struct K1Model {
        struct Impl;
        std::unique_ptr<Impl> impl;

        K1Model();
        ~K1Model();

        /// @brief Load the K1 URDF from @p path and print joint details.
        void load(const std::string& path);

        /// @brief Forward-kinematics to Head_2 from the current joint configuration.
        /// @return Htp — transform from the Head_2 pitch link to the Trunk (base).
        Eigen::Isometry3d compute_Htp(const std::unique_ptr<message::input::Sensors>& sensors) const;
    };

}  // namespace module::input

#endif  // MODULE_INPUT_K1SENSORS_K1MODEL_HPP
