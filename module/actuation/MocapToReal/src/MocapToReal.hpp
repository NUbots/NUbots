#ifndef MODULE_ACTUATION_MOCAPTOREAL_HPP
#define MODULE_ACTUATION_MOCAPTOREAL_HPP

#include <nuclear>

namespace module::actuation {

    class MocapToReal : public NUClear::Reactor {
    private:
        struct Config {
            bool output_to_servos = true;
            float servo_gain      = 10.0f;
            float servo_torque    = 100.0f;
            NUClear::clock::duration time_horizon{};
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
    };

}  // namespace module::actuation

#endif  // MODULE_ACTUATION_MOCAPTOREAL_HPP
