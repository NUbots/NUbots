#include "BallLocalisation.h"
#include <chrono>
#include "extension/Configuration.h"
#include "utility/time/time.h"

#include "message/localisation/FieldObject.h"
#include "message/vision/VisionObjects.h"
#include "message/support/FieldDescription.h"
#include "message/input/Sensors.h"


#include "utility/support/eigen_armadillo.h"

namespace module {
namespace localisation {

    using extension::Configuration;
    using utility::time::TimeDifferenceSeconds;
    using message::localisation::Ball;
    using message::support::FieldDescription;
    using message::input::Sensors;

    BallLocalisation::BallLocalisation(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment))
    , filter() {

        on<Configuration>("BallLocalisation.yaml").then([this] (const Configuration&) {
            auto message = std::make_unique<std::vector<Ball>>();
        	emit(message);
            emit(std::make_unique<Ball>());

            // Use configuration here from file BallLocalisation.yaml
        });

        /* To run at something like 100Hz that will call Time Update */
        on<Every<100, Per<std::chrono::seconds>>, Sync<BallLocalisation>>().then("BallLocalisation Time", [this] {
            auto curr_time = NUClear::clock::now();
            double seconds = TimeDifferenceSeconds(curr_time,last_time_update_time);
            last_time_update_time = curr_time;
            filter.timeUpdate(seconds);
        });

        /* To run whenever a ball has been detected */
        on<Trigger<std::vector<message::vision::Ball>>
         , With<FieldDescription>
         , With<Sensors>>().then([this](
            const std::vector<message::vision::Ball>& balls
            , const FieldDescription& field
            , const Sensors& sensors){

                double quality = 1.0;   // I don't know what quality should be used for
                if(balls.size() > 0){
                    /* Call Time Update first */
                    auto curr_time = NUClear::clock::now();
                    double seconds = TimeDifferenceSeconds(curr_time,last_time_update_time);
                    last_time_update_time = curr_time;
                    filter.timeUpdate(seconds);

                    /* Now call Measurement Update. Supports multiple measurement methods and will treat them as separate measurements */
                    for (auto& measurement : balls[0].measurements) {
                        quality *= filter.measurementUpdate(convert<double, 3, 1>(measurement.rBCc),convert<double, 3, 3>(measurement.covariance), field, sensors);
                    }
                }
        });
    }
}
}
