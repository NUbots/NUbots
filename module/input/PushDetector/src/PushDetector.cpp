/*
 * This file is part of NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2015 NUbots <nubots@nubots.net>
 */

#include "PushDetector.h"

#include "message/support/Configuration.h"
#include "utility/time/time.h"
#include "utility/nubugger/NUhelpers.h"
#include "message/input/Sensors.h"
#include "message/input/PushDetection.h"

namespace module 
{
namespace input 
{

    using message::support::Configuration;
    using message::input::Sensors;
    using message::input::PushDetection;
    
    using utility::input::ServoLoadModel;
    using utility::math::filter::UKF;
    using utility::time::TimeDifferenceSeconds;
    using utility::nubugger::graph;

/*=======================================================================================================*/
//      NAME: Push Detector
/*=======================================================================================================*/
    PushDetector::PushDetector(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)), loadFilters(), lastTimeUpdateTime(), DEBUG(false), DEBUG_ITER(0) 
    {

        // Configure push detector...
        on<Configuration>("PushDetector.yaml").then([this] (const Configuration& config) 
        {
            configure(config.config); 
        });

        // Sample a minimum of two sensor states to evaluate rates of change and load filtering...
        on<Last<2, Trigger<Sensors>>>().then([this] (const std::vector<std::shared_ptr<const Sensors>>& sensors) 
        {
            if(DEBUG) { log<NUClear::TRACE>("Push Detector - Enter Trigger(0)"); }
            if (sensors.size() < 2) 
            {
                return;
            }
            if(DEBUG) { log<NUClear::TRACE>("Push Detector - Enter Trigger(1)"); }

            // The acceleration amplitude for stationary postures is typically smaller than 0.40g...
            // The rotational rate amplitude for stationary postures typically is smaller than 1.0472 rads/s...
            // Evaluate thresholds for dynamic postures and abrupt noise (such as falling, jumping, etc.):
            // Acceleration amplitude = 3.0g...
            // Rotational rate = 3.49066 rads/sec...
            // Bending vs. Standing is considered at an offset of 0.610865 rads...

            arma::vec3 diff = sensors[0]->accelerometer - sensors[1]->accelerometer;
            arma::vec2 xzDiff = { diff(0), diff(2) };

            if (arma::norm(xzDiff) > 5) 
            {
                PushDetection det;
                det.forward = xzDiff(0) > 0;
                emit(std::make_unique<PushDetection>(det));
                emit(graph("PD: Detection", xzDiff(0) > 0 ? 1 : -1));
            } 
            else 
            {
                emit(graph("PD: Detection", 0));
            }

            // Load filtering...
            auto currentTime = NUClear::clock::now();
            double seconds = TimeDifferenceSeconds(currentTime, lastTimeUpdateTime);
            lastTimeUpdateTime = currentTime;

            for (uint i = 0; i < loadFilters.size(); i++) 
            {
                auto& filter = loadFilters[i];

                filter.timeUpdate(seconds);
                arma::mat cov = { 0.1 };
                arma::vec meas = { sensors[0]->servos[i].load };
                filter.measurementUpdate(meas, cov);
            }

            // Output filtered values to NUsight...
            arma::vec::fixed<20> filteredLoads;
            if(DEBUG) { log<NUClear::TRACE>("Push Detector - Printing Load Filtered Values(0)"); }
            for (uint i = 0; i < loadFilters.size(); i++) 
            {
                filteredLoads(i) = loadFilters[i].get()(0);
            }
            if(DEBUG) { log<NUClear::TRACE>("Push Detector - Printing Load Filtered Values(1)"); }
            emit(graph("PD: Filtered Loads", filteredLoads));
        });
    }
/*=======================================================================================================*/
//      INITIALISATION METHOD: Configuration
/*=======================================================================================================*/
    void PushDetector::configure(const YAML::Node& config)
    {
        auto& debug = config["debugging"];
        DEBUG = debug["enabled"].as<bool>();
    }        
}
}
