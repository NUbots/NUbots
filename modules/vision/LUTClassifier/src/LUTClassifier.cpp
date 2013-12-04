/*
 * This file is part of LUTClassifier.
 *
 * LUTClassifier is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * LUTClassifier is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with LUTClassifier.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "LUTClassifier.h"

namespace modules {
    namespace vision {

        using messages::input::Image;

        
        LUTClassifier::LUTClassifier(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
            on<Trigger<Image>>([this](const Image& image){

            	//std::vector<arma::vec> green_horizon_points = CalculateGreenHorizon(image);

            	//std::vector<int> scan_lines = GenerateScanLines(image,green_horizon_points);

            });
        }

        std::vector<arma::vec> LUTClassifier::CalculateGreenHorizon(const Image& img){
/*
        	//NEEDS KINEMATICS ! const Horizon& kin_hor = Last<1,KinematicsHorizon>;

		    size_t width = img.width();
		    size_t height = img.height();

		    //makes this fail-safe in the event of improper parameters
		    const int SPACING = std::max(VisionConstants::GREEN_HORIZON_SCAN_SPACING, 1U);
		    
		    // variable declarations    
		    std::vector<arma::vec> horizon_points;
		    std::vector<arma::vec> thrown_points;

	
		    int kin_hor_y;		

		    //For sampled pixel columns (vertical scans) sampled with period SPACING
		    for (int x = 0; x < width; x+=SPACING)
		    {
		        unsigned int green_top = 0;
		        unsigned int green_count = 0;

		        //Find kinematics horizon level for this vertical scan
		        
		        // kin_hor_y = kin_hor.findYFromX(x);
		        // //clamp green horizon values
		        // kin_hor_y = std::max(0, kin_hor_y);
		        // kin_hor_y = std::min(height-1, kin_hor_y);

		        //DUMMY CODE UNTIL KINEMATICS IMPLEMENTED. 
		        kin_hor_y = 0;		        //IE Search whole vertical strip
		        

		        //Search for green below the kinematics horizon
		        for (int y = kin_hor_y; y < height; y++) {

		            if (isPixelGreen(img, x, y)) {
		                if (green_count == 0) {
		                    green_top = y;
		                }
		                green_count++;
		                // if VER_THRESHOLD green pixels found, add point
		                if (green_count == VisionConstants::GREEN_HORIZON_MIN_GREEN_PIXELS) {
		                    vec v(2);
		                    v[0] = x;
		                    v[1] = green_top;
		                    horizon_points.push_back(v);
		                    break;
		                }
		            }
		            else {
		                // not green - reset
		                green_count = 0;
		            }
		        }
		    }

		    static int num_no_green = 0;
		    if(horizon_points.size() < 2) {
		        if(num_no_green < 150) {
		            num_no_green++;
		        }
		        else {
		            num_no_green = 0;
		            log<NUClear::ERROR>("150 FRAMES OF NO GREEN HORIZON FOUND - VERY POOR LUT");
		        }
		        horizon_points.clear();
		        vec v(2);
		        v[0] = 0;
		        v[1] = height-1;
		        horizon_points.push_back(v);
		        v[0] = width-1;
		        horizon_points.push_back(v);
		        
		        return horizon_points;
		    }

		    // provide blackboard the original set of scan points
		    vbb->setGreenHorizonScanPoints(horizon_points);

		    // statistical filter for green horizon points
		    double mean_y, std_dev_y;
		    accumulator_set<double, stats<tag::mean, tag::variance> > acc;

		    for(auto& p : horizon_points) {
		        if (p.[1] < height-1)     // if not at bottom of image
		            acc(p.[1]);
		    }

		    mean_y = mean(acc);
		    std_dev_y = sqrt(variance(acc)); 
		

		    std::vector<arma::vec>::iterator p = horizon_points.begin();
		    while(p < horizon_points.end()) {
		        if (p->y < mean_y - VisionConstants::GREEN_HORIZON_UPPER_THRESHOLD_MULT*std_dev_y) {
		            thrown_points.push_back(*p);
		            p = horizon_points.erase(p);
		        }
		        else {
		            p++;
		        }
		    }

		    DataWrapper::getInstance()->debugPublish(DBID_GREENHORIZON_THROWN, thrown_points);
		    horizon_points = upperConvexHull(horizon_points);
		    // set hull points
		    vbb->setGreenHullPoints(horizon_points);
		   */
		    return std::vector<arma::vec>();
        }
        
    }  // vision
}  // modules
