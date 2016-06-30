#include "BallLocalisation.h"

#include "message/support/Configuration.h"
#include "message/localisation/FieldObject.h"
#include "message/vision/VisionObjects.h"

namespace module {
namespace localisation {

    using message::support::Configuration;
    using message::localisation::Ball;

    BallLocalisation::BallLocalisation(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment))
    , filter() {

        on<Configuration>("BallLocalisation.yaml").then([this] (const Configuration&) {
        	auto message = std::make_unique<std::vector<Ball>>();
        	emit(message);
            // Use configuration here from file BallLocalisation.yaml
        });

        on<Trigger<std::vector<message::vision::Ball>>>().then([this](const std::vector<message::vision::Ball>& balls){
        	if(balls.size()>0){
	        	auto message = std::make_unique<std::vector<Ball>>();
	        	message->push_back(Ball());
	        	message->back().last_measurement_time = NUClear::clock::now();
	        	message->back().position = balls[0].torsoSpacePosition;
	        	emit(message);
        	}
        });
    }
}
}
