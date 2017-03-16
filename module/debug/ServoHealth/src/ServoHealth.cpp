#include "ServoHealth.h"

#include <fstream>

#include "extension/Configuration.h"
#include "extension/Script.h"

#include "message/support/ServoHealthTestData.h"
#include "message/platform/darwin/DarwinSensors.h"
#include "message/input/Sensors.h"

#include "utility/behaviour/Action.h"
#include "utility/input/LimbID.h"
#include "utility/input/ServoID.h"
#include "utility/platform/darwin/DarwinSensors.h"
#include "utility/nubugger/NUhelpers.h"

namespace module {
namespace debug {
            
    using extension::Configuration;
    using extension::ExecuteScriptByName;

    using message::input::Sensors;
    using message::platform::darwin::DarwinSensors;

    using utility::nubugger::graph;
    using utility::behaviour::RegisterAction;
    using message::support::ServoHealthTestData;
    using LimbID  = utility::input::LimbID;   
    using ServoID = utility::input::ServoID;
    using State = message::support::ServoHealthTestData::State;


    struct TestStart {};
    struct ScriptEnd {};
    struct TestStop  {};

    ServoHealth::ServoHealth(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment))
    , id(size_t(this) * size_t(this) - size_t(this))
    , state(State::INITIALISE)
    , fallen_angle(0)
    {



        on<Configuration>("ServoHealth.yaml").then([this] (const Configuration& config){    
            double fallenAngleConfig = config["fallen_angle"].as<double>();
            test_loops = config["test_loops"].as<int>();
            fallen_angle = cos(fallenAngleConfig);    
        });

        
        emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction {
            id,
            "ServoTest",
            { std::pair<float, std::set<LimbID>>(0, { LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM, LimbID::HEAD }) },
            [this] (const std::set<LimbID>&) {
                emit<Scope::DELAY>(std::make_unique<TestStart>(), std::chrono::milliseconds(1500)); //TODO find fix for dirty hack
            },
            [this] (const std::set<LimbID>&) {
                emit(std::make_unique<TestStop>());
            },
            [this] (const std::set<ServoID>&) {
                emit(std::make_unique<ScriptEnd>());
            }
        }));

        on<Trigger<DarwinSensors>>().then("Log Servo Data", [this] (const DarwinSensors& sensors) {

            // If this is a testing state
            if (state != State::INITIALISE 
                && state != State::LAY_DOWN_1
                && state != State::MOVE_LEGS
                && state != State::LAY_DOWN_2
                && state != State::FINISHED) {

                auto data = std::make_unique<ServoHealthTestData>();
                data->state = state;
                data->sensors = sensors;

                emit(data);
            }
        });

        on<Trigger<TestStart>, With<Sensors>>().then([this] (const Sensors& sensors) {
            double duration1 = 1.0;

            // Set up initial state.
          
            if(fabs(sensors.world(2, 2)) < fallen_angle) {
                if (sensors.world(0, 2) < 0.0) {
                    emit(std::make_unique<ExecuteScriptByName>(id, std::vector<std::string>({"Relax.yaml","Relax.yaml","StandUpFront.yaml","YogaSplit1.yaml"})));
                    emit(std::make_unique<ExecuteScriptByName>(id, "YogaSplit1.yaml", duration1));
                    log<NUClear::ERROR>("FRONT");

                }
                else {
                    emit(std::make_unique<ExecuteScriptByName>(id, std::vector<std::string>({"Relax.yaml","Relax.yaml","StandUpBack.yaml","YogaSplit1.yaml"})));
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

        on<Trigger<ScriptEnd>>().then([this]  {
            //Durations to execute scripts at
            double duration1 = 2.0;

            switch(State::Value(state)) {
                case State::INITIALISE: {                    
                    counter = 0;
                    state = State::TEST_ARMS_PITCH;
                    emit(std::make_unique<ExecuteScriptByName>(id, "YogaSplit2.yaml", duration1));
                } break;

                case State::TEST_ARMS_PITCH: {
                    if (++counter > test_loops) {
                        state = State::TEST_ARMS_ROLL;
                        counter = 0;
                        emit(std::make_unique<ExecuteScriptByName>(id, "YogaSplit3.yaml", duration1));
                    }
                    else {
                        emit(std::make_unique<ExecuteScriptByName>(id, "YogaSplit2.yaml", duration1));
                    } 
                } break;

                case State::TEST_ARMS_ROLL: {
                    if (++counter > test_loops) {
                        state = State::TEST_HEAD;
                        counter = 0;
                        emit(std::make_unique<ExecuteScriptByName>(id, "YogaSplit4.yaml", duration1));
                    }
                    else {
                        emit(std::make_unique<ExecuteScriptByName>(id, "YogaSplit3.yaml", duration1));
                    } 
                } break;
                
                case State::TEST_HEAD: {
                    if (++counter > test_loops) {
                        state = State::LAY_DOWN_1;
                        counter = 0;
                        emit(std::make_unique<ExecuteScriptByName>(id, "YogaSplit5.yaml", duration1));
                    }
                    else {
                        emit(std::make_unique<ExecuteScriptByName>(id, "YogaSplit4.yaml", duration1));
                    } 
                } break;

                case State::LAY_DOWN_1: {
                    counter = 0;
                    state = State::TEST_HIP_ROLL;
                    emit(std::make_unique<ExecuteScriptByName>(id, "YogaSplit6.yaml", duration1));
                } break;

                case State::TEST_HIP_ROLL: {
                    if (++counter > test_loops) {
                        state = State::TEST_HIP_YAW;
                        counter = 0;
                        emit(std::make_unique<ExecuteScriptByName>(id, "YogaSplit12.yaml", duration1));
                    }
                    else {
                        emit(std::make_unique<ExecuteScriptByName>(id, "YogaSplit6.yaml", duration1));
                    } 
                } break;
                
                case State::TEST_HIP_YAW: {
                    if (++counter > test_loops) {
                        state = State::TEST_ANKLES;
                        counter = 0;
                        emit(std::make_unique<ExecuteScriptByName>(id, "YogaSplit7.yaml", duration1));
                    }
                    else {
                        emit(std::make_unique<ExecuteScriptByName>(id, "YogaSplit12.yaml", duration1));
                    } 
                } break;
                
                case State::TEST_ANKLES: {
                    if (++counter > test_loops) {
                        state = State::MOVE_LEGS;
                        counter = 0;
                        emit(std::make_unique<ExecuteScriptByName>(id, "YogaSplit8.yaml", duration1));
                    }
                    else {
                        emit(std::make_unique<ExecuteScriptByName>(id, "YogaSplit7.yaml", duration1));
                    } 
                } break;
                
                case State::MOVE_LEGS: {
                    counter = 0;
                    state = State::TEST_KNEES;
                    emit(std::make_unique<ExecuteScriptByName>(id, "YogaSplit13.yaml", duration1));
                } break;

                case State::TEST_KNEES: {
                    if (++counter > test_loops) {
                        state = State::TEST_LEGS_PITCH;
                        counter = 0;
                        emit(std::make_unique<ExecuteScriptByName>(id, "YogaSplit9.yaml", duration1));
                    }
                    else {
                        emit(std::make_unique<ExecuteScriptByName>(id, "YogaSplit13.yaml", duration1));
                    } 
                } break;
                
                case State::TEST_LEGS_PITCH: {
                    if (++counter > test_loops) {
                        state = State::LAY_DOWN_2;
                        counter = 0;
                        emit(std::make_unique<ExecuteScriptByName>(id, "YogaSplit10.yaml", duration1));
                    }
                    else {
                        emit(std::make_unique<ExecuteScriptByName>(id, "YogaSplit9.yaml", duration1));
                    } 
                } break;
                
                case State::LAY_DOWN_2: {
                    counter = 0;
                    state = State::FINISHED;
                    emit(std::make_unique<ExecuteScriptByName>(id, std::vector<std::string>({"StandUpFront.yaml","Stand.yaml"})));
                } break;

                case State::FINISHED:
                {
                    //Add time, date, and robot ID stamp to filename
                    //std::time_t now = std::chrono::system_clock::to_time_t(NUClear::clock::now());
                    //char buf[sizeof "2011-10-08-07-07-09"];
                    //strftime(buf, sizeof buf, "%F-%H-%M-%S", gmtime(&now));
                    //char hostname[255];
                    //gethostname(hostname, 255);


                    //std::ofstream ofs("histogram" + std::string(hostname) + std::string(buf) + ".csv");
                    //if(!ofs.is_open()){
                    //    log("Failed to open histogram file");
                    powerplant.shutdown();    
                } break;


                default:
                {
                    emit(std::make_unique<TestStart>());
                    break;
                }
            }
        });
    }
}
}