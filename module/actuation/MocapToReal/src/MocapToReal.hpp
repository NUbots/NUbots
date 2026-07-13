#ifndef MODULE_ACTUATION_MOCAPTOREAL_HPP
#define MODULE_ACTUATION_MOCAPTOREAL_HPP

#include <array>
#include <nuclear>

namespace module::actuation {

    class MocapToReal : public NUClear::Reactor {
    private:
        struct Config {
            bool output_to_servos = true;
            float servo_gain      = 10.0f;
            float servo_torque    = 100.0f;
            NUClear::clock::duration time_horizon{};
            /// Hinge axis, in the parent bone's local frame, used to extract a signed bend angle
            std::array<float, 3> hinge_axis = {1.0f, 0.0f, 0.0f};
            /// Per-joint signs mapping mocap bend angles onto message-space servo directions
            float r_elbow_sign = -1.0f;
            float l_elbow_sign = -1.0f;
            float r_knee_sign  = 1.0f;
            float l_knee_sign  = 1.0f;
            float r_ankle_sign = -1.0f;
            float l_ankle_sign = -1.0f;
        } cfg;

    public:
        explicit MocapToReal(std::unique_ptr<NUClear::Environment> environment);

        typedef struct {
            float x;
            float y;
            float z;
            float t;
        } qRot;

        // Quaternion helpers
        qRot qInvert(qRot);
        qRot qMultiply(qRot, qRot);
        float angleBetween(qRot, qRot);
        /// Signed twist of the relative rotation from qA to qB about `axis` (parent frame), in [-pi, pi]
        float signedAngleBetween(qRot qA, qRot qB, const std::array<float, 3>& axis);
    };

}  // namespace module::actuation

#endif  // MODULE_ACTUATION_MOCAPTOREAL_HPP
