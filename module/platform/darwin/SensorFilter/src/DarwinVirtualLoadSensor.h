/*
 * This file is part of the NUbots Codebase.
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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_PLATFORM_DARWIN_DARWINVIRTUALLOADSENSOR_H
#define MODULES_PLATFORM_DARWIN_DARWINVIRTUALLOADSENSOR_H

#include "utility/math/matrix/Transform3D.h"
#include "utility/math/matrix/Rotation3D.h"


namespace module {
    namespace platform {
        namespace darwin {
        
            class DarwinVirtualLoadSensor {
                //this implements a linear model (trained by logistic regression) with a bayes filter on the output
                private:
                    double currentNoise;
                    double noiseFactor;
                    double intercept;
                    double certaintyThreshold;
                    arma::vec featureWeights;
                public:
                    double state = 0.5;
                    
                    DarwinVirtualLoadSensor();
                    
                    DarwinVirtualLoadSensor(arma::vec fWeights, 
                                            double interc,
                                            double nFactor,
                                            double certaintyThresh) {
                                            
                        noiseFactor = nFactor;
                        currentNoise = 2 * nFactor;
                        certaintyThreshold = certaintyThresh;
                        
                        intercept = interc;
                        featureWeights = fWeights;
                    }
                    
                    bool updateFoot(arma::vec legMotors) {
                        
                        //create the feature vector (a few nonlinear combinations of variables)
                        arma::vec features(4*legMotors.size()-3);
                        features.rows(0,legMotors.size()-1) = legMotors;
                        features.rows(legMotors.size(),2*legMotors.size()-1) = legMotors % arma::square(legMotors);
                        features.rows(2*legMotors.size(),2*legMotors.size()-2) = legMotors[0] * legMotors.rows(1,legMotors.size()-1);
                        features.rows(3*legMotors.size()-1,2*legMotors.size()-4) = legMotors[1] * legMotors.rows(2,legMotors.size()-1);
                        
                        //do the prediction
                        double clss = arma::dot(features, featureWeights) + intercept > 0.;
                        
                        //do the bayes update (1D kalman filter thing)
                        double k = currentNoise / (currentNoise + noiseFactor);
                        state += k*(clss-state);
                        currentNoise *= 1-k;
                        
                        return state > certaintyThreshold;
                    }
            
            
            };
        }
    }
}

#endif
