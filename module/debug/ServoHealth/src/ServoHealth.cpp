#include "ServoHealth.h"

#include "message/support/Configuration.h"
#include "message/platform/darwin/DarwinSensors.h"
#include "message/behaviour/Action.h"
#include "message/input/ServoID.h"
#include "message/motion/Script.h"
#include "message/input/LimbID.h"





namespace module {
namespace debug {
            
    using message::support::Configuration;
    using message::platform::darwin::DarwinSensors;
    using message::input::ServoID;
    using message::input::LimbID;   
    using message::motion::ExecuteScriptByName;
    using message::behaviour::RegisterAction;



    ServoHealth::ServoHealth(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment))
    , id(size_t(this) * size_t(this) - size_t(this))
    , loadHealth()
    {

        on<Configuration>("ServoHealth.yaml").then([this] /*(const Configuration& config)*/ {
            // Use configuration here from file ServoHealth.yaml
        });

        emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction {
            id,
            "ServoTest",
            { std::pair<float, std::set<LimbID>>(0, { LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM }) },
            [this] (const std::set<LimbID>&) {
                
            },
            [this] (const std::set<LimbID>&) {
                
            },
            [this] (const std::set<ServoID>&) {
            }
        }));


        on<Last<2, Trigger<DarwinSensors>>>().then("Servo Health Check", [this] (const std::vector<std::shared_ptr<const DarwinSensors>>& sensors) {
        	if (sensors.size() < 2) {
                return;
            }
        	for(ServoID i = ServoID(0); i < ServoID::NUMBER_OF_SERVOS; i = ServoID(int(i)+1)){
        		loadHealth[i][((sensors[0]->servo[i].presentPosition + M_PI)/(2*M_PI))*4095] = abs(sensors[0]->servo[i].load - sensors[1]->servo[i].load);
        	}
        });

        on<Every<7, Per<std::chrono::seconds>>, Single>().then([this] {
        	int maxIndex = 0;
        	for (int i = 1; i < 4096; i++)
        	{
        		if (loadHealth[ServoID(12)][maxIndex] < loadHealth[ServoID(12)][i])
        			maxIndex = i;
        	}
			std::cout << "Max Position at" << maxIndex <<  ": " << loadHealth[ServoID(12)][maxIndex] << "\n\r";
			emit(std::make_unique<ExecuteScriptByName>(id, std::vector<std::string>({"Stand.yaml", "Zombie.yaml", "Stand.yaml"})));
        });
    }
}
}
