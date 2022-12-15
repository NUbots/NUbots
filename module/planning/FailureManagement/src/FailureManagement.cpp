#include "FailureManagement.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/behaviour/State.hpp"
#include "message/input/Sensors.hpp"
#include "message/planning/Recover.hpp"

namespace module::planning {

    using extension::Configuration;

    enum class State { STABLE, UNSTABLE, FALLING };

    double smooth(double value, double new_value, double alpha) {
        return alpha * value + (1.0 - alpha) * new_value;
    }

    FailureManagement::FailureManagement(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("FailureManagement.yaml").then([this](const Configuration& config) {
            // Use configuration here from file FailureManagement.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.gyro.mag.mean       = config["gyro"]["mag"]["mean"].as<Expression>();
            cfg.gyro.mag.unstable   = config["gyro"]["mag"]["unstable"].as<Expression>();
            cfg.gyro.mag.falling    = config["gyro"]["mag"]["falling"].as<Expression>();
            cfg.gyro.mag.smoothing  = config["gyro"]["mag"]["smoothing"].as<Expression>();
            cfg.acc.mag.mean        = config["accelerometer"]["mag"]["mean"].as<Expression>();
            cfg.acc.mag.unstable    = config["accelerometer"]["mag"]["unstable"].as<Expression>();
            cfg.acc.mag.falling     = config["accelerometer"]["mag"]["falling"].as<Expression>();
            cfg.acc.mag.smoothing   = config["accelerometer"]["mag"]["smoothing"].as<Expression>();
            cfg.acc.angle.unstable  = config["accelerometer"]["angle"]["unstable"].as<Expression>();
            cfg.acc.angle.falling   = config["accelerometer"]["angle"]["falling"].as<Expression>();
            cfg.acc.angle.smoothing = config["accelerometer"]["angle"]["smoothing"].as<Expression>();
            cfg.getup.angle         = config["getup"]["angle"].as<Expression>();
        });

        on<Provide<Recover>, Trigger<Sensors>>().then([this](const RunInfo& info, const Sensors& sensors) {
            switch (info.run_reason) {
                case RunInfo::OTHER_TRIGGER: {
                    // OTHER_TRIGGER means we ran because of a sensors update

                    // Smooth the values we use to determine if we are falling
                    gyro.mag  = smooth(gyro.mag,  // Sum of roll and pitch rotation
                                      std::abs(sensors.gyro.x()) + std::abs(sensors.gyro.y() - cfg.gyro.mag.mean),
                                      cfg.gyro.mag.smoothing);
                    acc.mag   = smooth(acc.mag,  // Magnitude of the acceleration vector without gravity
                                     std::abs(sensors.accelerometer.norm() - cfg.acc.mag.mean),
                                     cfg.acc.mag.smoothing);
                    acc.angle = smooth(acc.angle,  // Angle of the acceleration vector
                                       1.0 - std::abs(sensors.accelerometer.normalized().z()),
                                       cfg.acc.angle.smoothing);

                    // If we are getting up we ignore what's going on until we are done
                    if (getting_up) {
                        emit(std::make_unique<Idle>());
                    }
                    else {

                        // Check if we are stable according to each sensor
                        State gyro_mag_state  = gyro.mag < cfg.gyro.mag.unstable  ? State::STABLE
                                                : gyro.mag < cfg.gyro.mag.falling ? State::UNSTABLE
                                                                                  : State::FALLING;
                        State acc_mag_state   = acc.mag < cfg.acc.mag.unstable  ? State::STABLE
                                                : acc.mag < cfg.acc.mag.falling ? State::UNSTABLE
                                                                                : State::FALLING;
                        State acc_angle_state = acc.angle < cfg.acc.angle.unstable  ? State::STABLE
                                                : acc.angle < cfg.acc.angle.falling ? State::UNSTABLE
                                                                                    : State::FALLING;

                        // Falling if at least two of the three checks are unstable or if any one of them is falling
                        bool falling = (gyro_mag_state == State::FALLING || acc_mag_state == State::FALLING
                                        || acc_angle_state == State::FALLING)
                                       || (gyro_mag_state == State::UNSTABLE && acc_mag_state == State::UNSTABLE)
                                       || (gyro_mag_state == State::UNSTABLE && acc_angle_state == State::UNSTABLE)
                                       || (acc_mag_state == State::UNSTABLE && acc_angle_state == State::UNSTABLE);

                        // We are falling
                        if (falling) {
                            // TODO emit stability falling
                            emit(std::make_unique<Stability>(Stability::FALLING));
                            emit<Script>(std::make_unique<Body>(), "Relax.yaml");
                        }
                        // We can get up if all our sensors are in the recovery range and we are lying down
                        else if (gyro.mag < cfg.gyro.mag.recovery && acc.mag < cfg.acc.mag.recovery
                                 && acc.angle < cfg.getup.angle) {

                            emit(std::make_unique<Stability>(Stability::FALLEN));
                            // Now we work out which way we are facing, splitting into four quadrants
                            // using the accelerometer to get the four diagonals
                            bool a = sensors.accelerometer.x() > sensors.accelerometer.y();
                            bool b = sensors.accelerometer.x() > -sensors.accelerometer.y();

                            getting_up = true;
                            if (a && b) {
                                emit<Script>(std::make_unique<Body>(), "GetUpFront.yaml");
                            }
                            else if (a && !b) {
                                // TODO not sure if this is the right way to roll
                                emit<Script>(std::make_unique<Body>(), "RollLeft.yaml");
                            }
                            else if (!a && b) {
                                // TODO not sure if this is the right way to roll
                                emit<Script>(std::make_unique<Body>(), "RollRight.yaml");
                            }
                            else if (!a && !b) {
                                emit<Script>(std::make_unique<Body>(), "GetUpBack.yaml");
                            }
                        }
                    }


                } break;
                case RunInfo::SUBTASK_DONE: {
                    // SUBTASK_DONE means the script we executed finished running

                    // Finished getting up, so we no longer have a subtask
                    if (getting_up) {
                        // TODO what if we failed to get up?
                        getting_up = false;
                        emit(std::make_unique<Stability>(Stability::STANDING));
                    }
                    // Finished a relax script, just emit idle
                    else {
                        emit<Task>(std::make_unique<Idle>());
                    }
                } break;
            }
        });
    }

}  // namespace module::planning
