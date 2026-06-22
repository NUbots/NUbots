// This file intentionally contains ALL tinyrobotics template instantiations for
// the K1 robot.  Keeping them isolated here prevents the LLVM/clang OOM that
// occurs when the heavy template code is mixed with the large Eigen sensor
// processing translation unit (K1Sensors.cpp).

#include "k1_model.hpp"

// Heavy headers — only included in this one TU
#include <tinyrobotics/kinematics.hpp>
#include <tinyrobotics/parser.hpp>

#include "message/input/Sensors.hpp"
#include "utility/actuation/tinyrobotics.hpp"

namespace module::input {

    static constexpr int n_servos = 22;

    struct K1Model::Impl {
        tinyrobotics::Model<double, n_servos> model;
    };

    K1Model::K1Model() : impl(std::make_unique<Impl>()) {}
    K1Model::~K1Model() = default;

    void K1Model::load(const std::string& path) {
        impl->model = tinyrobotics::import_urdf<double, n_servos>(path);
        impl->model.show_details();
    }

    Eigen::Isometry3d K1Model::compute_Htp(const std::unique_ptr<message::input::Sensors>& sensors) const {
        using utility::actuation::tinyrobotics::sensors_to_configuration;
        const Eigen::Matrix<double, n_servos, 1> q = sensors_to_configuration<double, n_servos>(sensors);
        return tinyrobotics::forward_kinematics(impl->model, q, std::string("Head_2"));
    }

}  // namespace module::input
