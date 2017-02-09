#include "ServoHealth.h"

#include "extension/Configuration.h"
#include "extension/Script.h"

#include "message/platform/darwin/DarwinSensors.h"

#include "utility/behaviour/Action.h"
#include "utility/input/LimbID.h"
#include "utility/input/ServoID.h"
#include "utility/platform/darwin/DarwinSensors.h"

namespace module {
namespace debug {
            
    using extension::Configuration;
    using extension::ExecuteScriptByName;

    using message::platform::darwin::DarwinSensors;

    using utility::behaviour::RegisterAction;
    using LimbID  = utility::input::LimbID;   
    using ServoID = utility::input::ServoID;


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
                auto firstServo  = utility::platform::darwin::getDarwinServo(i, *sensors[0]);
                auto secondServo = utility::platform::darwin::getDarwinServo(i, *sensors[1]);

        		loadHealth[i][((firstServo.presentPosition + M_PI) / (2.0 * M_PI)) * 4095] = abs(firstServo.load - secondServo.load);
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
