#include "ServoHealth.h"

#include <fstream>

#include "extension/Configuration.h"
#include "extension/Script.h"

#include "message/input/Sensors.h"
#include "message/platform/darwin/DarwinSensors.h"
#include "message/support/ServoHealthTestData.h"

#include "utility/behaviour/Action.h"
#include "utility/input/LimbID.h"
#include "utility/input/ServoID.h"
#include "utility/nusight/NUhelpers.h"
#include "utility/platform/darwin/DarwinSensors.h"

namespace module {
namespace debug {

    using extension::Configuration;
    using extension::ExecuteScriptByName;

    using message::input::Sensors;
    using message::platform::darwin::DarwinSensors;

    using message::support::ServoHealthTestData;
    using utility::behaviour::RegisterAction;
    using utility::nusight::graph;
    using LimbID  = utility::input::LimbID;
    using ServoID = utility::input::ServoID;
    using State   = message::support::ServoHealthTestData::State;


    struct TestStart {};
    struct ScriptEnd {};
    struct TestStop {};

    ServoHealth::ServoHealth(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , id(size_t(this) * size_t(this) - size_t(this))
        , state(State::INITIALISE)
        , fallen_angle(0) {


        on<Configuration>("ServoHealth.yaml").then([this](const Configuration& config) {
            double fallenAngleConfig = config["fallen_angle"].as<double>();
            test_loops               = config["test_loops"].as<int>();
            fallen_angle             = cos(fallenAngleConfig);
        });


        emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction{
            id,
            "ServoTest",
            {std::pair<float, std::set<LimbID>>(
                0, {LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM, LimbID::HEAD})},
            [this](const std::set<LimbID>&) {
                emit<Scope::DELAY>(std::make_unique<TestStart>(),
                                   std::chrono::milliseconds(1500));  // TODO find fix for dirty hack
            },
            [this](const std::set<LimbID>&) { emit(std::make_unique<TestStop>()); },
            [this](const std::set<ServoID>&) { emit(std::make_unique<ScriptEnd>()); }}));

        on<Trigger<DarwinSensors>>().then("Log Servo Data", [this](const DarwinSensors& sensors) {
            // If this is a testing state
            if (state != State::INITIALISE && state != State::MOVE_1 && state != State::MOVE_2
                && state != State::SHOULDER_MOVE_1 && state != State::MOVE_3 && state != State::MOVE_4
                && state != State::LAYDOWN && state != State::HIP_MOVE_1 && state != State::HIP_MOVE_2
                && state != State::ANKLE_MOVE && state != State::KNEE_MOVE && state != State::KNEE_MOVE_2
                && state != State::LAYDOWN_2 && state != State::FINISHED) {

                auto data     = std::make_unique<ServoHealthTestData>();
                data->state   = state;
                data->sensors = sensors;

                emit(data);
            }
        });

        on<Trigger<TestStart>, With<Sensors>>().then([this](const Sensors& sensors) {
            double duration1 = 1.0;

            // Set up initial state.

            if (fabs(sensors.world(2, 2)) < fallen_angle) {
                if (sensors.world(0, 2) < 0.0) {
                    emit(std::make_unique<ExecuteScriptByName>(
                        id,
                        std::vector<std::string>(
                            {"Relax.yaml", "Relax.yaml", "StandUpFront.yaml", "YogaSplit1.yaml"})));
                    emit(std::make_unique<ExecuteScriptByName>(id, "YogaSplit1.yaml", duration1));
                    log<NUClear::ERROR>("FRONT");
                }
                else {
                    emit(std::make_unique<ExecuteScriptByName>(
                        id,
                        std::vector<std::string>({"Relax.yaml", "Relax.yaml", "StandUpBack.yaml", "YogaSplit1.yaml"})));
                    emit(std::make_unique<ExecuteScriptByName>(id, "YogaSplit1.yaml", duration1));
                    log<NUClear::ERROR>("BACK");
                }
            }
            else {
                emit(std::make_unique<ExecuteScriptByName>(id, "YogaSplit1.yaml", duration1));
                emit(std::make_unique<ExecuteScriptByName>(id, "YogaSplit1.yaml", duration1));
                log<NUClear::ERROR>("I'M UP MAN");
            }
            state = State::INITIALISE;
        });

        on<Trigger<TestStop>>().then([this] {

        });

        on<Trigger<ScriptEnd>>().then([this] {
            // Durations to execute scripts at
            double duration1 = 2.0;

            switch (State::Value(state)) {
                case State::INITIALISE: {
                    counter = 0;
                    state   = State::MOVE_1;
                    emit(std::make_unique<ExecuteScriptByName>(id, "Yoga_Move_1.yaml", duration1));
                } break;

                case State::MOVE_1: {
                    counter = 0;
                    state   = State::ELBOW;
                    emit(std::make_unique<ExecuteScriptByName>(id, "Yoga_Elbow.yaml", duration1));
                } break;

                case State::ELBOW: {
                    if (++counter > test_loops) {
                        state   = State::MOVE_2;
                        counter = 0;
                        emit(std::make_unique<ExecuteScriptByName>(id, "Yoga_Move_2.yaml", duration1));
                    }
                    else {
                        emit(std::make_unique<ExecuteScriptByName>(id, "Yoga_Elbow.yaml", duration1));
                    }
                } break;
                case State::MOVE_2: {
                    counter = 0;
                    state   = State::SHOULDER_PITCH;
                    emit(std::make_unique<ExecuteScriptByName>(id, "Yoga_Shoulder_Pitch.yaml", duration1));
                } break;

                case State::SHOULDER_PITCH: {
                    if (++counter > test_loops) {
                        state   = State::SHOULDER_MOVE_1;
                        counter = 0;
                        emit(std::make_unique<ExecuteScriptByName>(id, "Yoga_Shoulder_Move_1.yaml", duration1));
                    }
                    else {
                        emit(std::make_unique<ExecuteScriptByName>(id, "Yoga_Shoulder_Pitch.yaml", duration1));
                    }
                } break;
                case State::SHOULDER_MOVE_1: {
                    counter = 0;
                    state   = State::SHOULDER_ROLL;
                    emit(std::make_unique<ExecuteScriptByName>(id, "Yoga_Shoulder_Roll.yaml", duration1));
                } break;

                case State::SHOULDER_ROLL: {
                    if (++counter > test_loops) {
                        state   = State::MOVE_3;
                        counter = 0;
                        emit(std::make_unique<ExecuteScriptByName>(id, "Yoga_Move_3.yaml", duration1));
                    }
                    else {
                        emit(std::make_unique<ExecuteScriptByName>(id, "Yoga_Shoulder_Roll.yaml", duration1));
                    }
                } break;

                case State::MOVE_3: {
                    counter = 0;
                    state   = State::HEAD_PITCH;
                    emit(std::make_unique<ExecuteScriptByName>(id, "Yoga_Head_Pitch.yaml", duration1));
                } break;

                case State::HEAD_PITCH: {
                    if (++counter > test_loops) {
                        state   = State::MOVE_4;
                        counter = 0;
                        emit(std::make_unique<ExecuteScriptByName>(id, "Yoga_Move_4.yaml", duration1));
                    }
                    else {
                        emit(std::make_unique<ExecuteScriptByName>(id, "Yoga_Head_Pitch.yaml", duration1));
                    }
                } break;
                case State::MOVE_4: {
                    counter = 0;
                    state   = State::HEAD_YAW;
                    emit(std::make_unique<ExecuteScriptByName>(id, "Yoga_Head_Yaw.yaml", duration1));
                } break;

                case State::HEAD_YAW: {
                    if (++counter > test_loops) {
                        state   = State::LAYDOWN;
                        counter = 0;
                        emit(std::make_unique<ExecuteScriptByName>(id, "Yoga_Laydown.yaml", duration1));
                    }
                    else {
                        emit(std::make_unique<ExecuteScriptByName>(id, "Yoga_Head_Yaw.yaml", duration1));
                    }
                } break;
                case State::LAYDOWN: {
                    counter = 0;
                    state   = State::HIP_ROLL;
                    emit(std::make_unique<ExecuteScriptByName>(id, "Yoga_Hip_Roll.yaml", duration1));
                } break;

                case State::HIP_ROLL: {
                    if (++counter > test_loops) {
                        state   = State::HIP_MOVE_1;
                        counter = 0;
                        emit(std::make_unique<ExecuteScriptByName>(id, "Yoga_Hip_Move_1.yaml", duration1));
                    }
                    else {
                        emit(std::make_unique<ExecuteScriptByName>(id, "Yoga_Hip_Roll.yaml", duration1));
                    }
                } break;

                case State::HIP_MOVE_1: {
                    counter = 0;
                    state   = State::HIP_YAW;
                    emit(std::make_unique<ExecuteScriptByName>(id, "Yoga_Hip_Yaw.yaml", duration1));
                } break;

                case State::HIP_YAW: {
                    if (++counter > test_loops) {
                        state   = State::HIP_MOVE_2;
                        counter = 0;
                        emit(std::make_unique<ExecuteScriptByName>(id, "Yoga_Hip_Move_2.yaml", duration1));
                    }
                    else {
                        emit(std::make_unique<ExecuteScriptByName>(id, "Yoga_Hip_Yaw.yaml", duration1));
                    }
                } break;
                case State::HIP_MOVE_2: {
                    counter = 0;
                    state   = State::ANKLE_PITCH;
                    emit(std::make_unique<ExecuteScriptByName>(id, "Yoga_Ankle_Pitch.yaml", duration1));
                } break;

                case State::ANKLE_PITCH: {
                    if (++counter > test_loops) {
                        state   = State::ANKLE_MOVE;
                        counter = 0;
                        emit(std::make_unique<ExecuteScriptByName>(id, "Yoga_Ankle_Move.yaml", duration1));
                    }
                    else {
                        emit(std::make_unique<ExecuteScriptByName>(id, "Yoga_Ankle_Pitch.yaml", duration1));
                    }
                } break;

                case State::ANKLE_MOVE: {
                    counter = 0;
                    state   = State::ANKLE_ROLL;
                    emit(std::make_unique<ExecuteScriptByName>(id, "Yoga_Ankle_Roll.yaml", duration1));
                } break;

                case State::ANKLE_ROLL: {
                    if (++counter > test_loops) {
                        state   = State::KNEE_MOVE;
                        counter = 0;
                        emit(std::make_unique<ExecuteScriptByName>(id, "Yoga_Knee_Move.yaml", duration1));
                    }
                    else {
                        emit(std::make_unique<ExecuteScriptByName>(id, "Yoga_Ankle_Roll.yaml", duration1));
                    }
                } break;

                case State::KNEE_MOVE: {
                    counter = 0;
                    state   = State::KNEE;
                    emit(std::make_unique<ExecuteScriptByName>(id, "Yoga_Knee.yaml", duration1));
                } break;

                case State::KNEE: {
                    if (++counter > test_loops) {
                        state   = State::KNEE_MOVE_2;
                        counter = 0;
                        emit(std::make_unique<ExecuteScriptByName>(id, "Yoga_Knee_Move_2.yaml", duration1));
                    }
                    else {
                        emit(std::make_unique<ExecuteScriptByName>(id, "Yoga_Knee.yaml", duration1));
                    }
                } break;

                case State::KNEE_MOVE_2: {
                    counter = 0;
                    state   = State::HIP_PITCH;
                    emit(std::make_unique<ExecuteScriptByName>(id, "Yoga_Hip_Pitch.yaml", duration1));
                } break;
                case State::HIP_PITCH: {
                    if (++counter > test_loops) {
                        state   = State::LAYDOWN_2;
                        counter = 0;
                        emit(std::make_unique<ExecuteScriptByName>(id, "Yoga_Laydown_2.yaml", duration1));
                    }
                    else {
                        emit(std::make_unique<ExecuteScriptByName>(id, "Yoga_Hip_Pitch.yaml", duration1));
                    }
                } break;

                case State::LAYDOWN_2: {
                    counter = 0;
                    state   = State::FINISHED;
                    emit(std::make_unique<ExecuteScriptByName>(
                        id, std::vector<std::string>({"StandUpFront.yaml", "Stand.yaml"})));
                    emit(std::make_unique<ExecuteScriptByName>(id, "Stand.yaml", duration1));
                } break;

                case State::FINISHED: {
                    // Add time, date, and robot ID stamp to filename
                    // std::time_t now = std::chrono::system_clock::to_time_t(NUClear::clock::now());
                    // char buf[sizeof "2011-10-08-07-07-09"];
                    // strftime(buf, sizeof buf, "%F-%H-%M-%S", gmtime(&now));
                    // char hostname[255];
                    // gethostname(hostname, 255);


                    // std::ofstream ofs("histogram" + std::string(hostname) + std::string(buf) + ".csv");
                    // if(!ofs.is_open()){
                    //    log("Failed to open histogram file");
                    // powerplant.shutdown();
                } break;


                default: {
                    emit(std::make_unique<TestStart>());
                    break;
                }
            }
        });
    }
}  // namespace debug
}  // namespace module
