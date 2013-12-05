/*
 * This file is part of ScanLines.
 *
 * ScanLines is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ScanLines is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ScanLines.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "GreenHorizon.h"

namespace modules {
    namespace vision {

        using messages::input::Image;
        using messages::support::Configuration;
        
        GreenHorizon::GreenHorizon() :
            GREEN_HORIZON_SCAN_SPACING(),
            GREEN_HORIZON_MIN_GREEN_PIXELS(),
            GREEN_HORIZON_UPPER_THRESHOLD_MULT(){}


        std::vector<arma::vec> GreenHorizon::calculateGreenHorizon(const Image& img, const LookUpTable& LUT) {

        	//NEEDS KINEMATICS ! const Horizon& kin_hor = Last<1,KinematicsHorizon>;

		    size_t width = img.width();
		    size_t height = img.height();

		    //makes this fail-safe in the event of improper parameters
		    const int SPACING = std::max(GREEN_HORIZON_SCAN_SPACING, 1U);
		    
		    // variable declarations    
		    std::vector<arma::vec> horizon_points;
		    std::vector<arma::vec> thrown_points;
	
		    int kin_hor_y;		
		    //For sampled pixel columns (vertical scans) sampled with period SPACING
		    for (size_t x = 0; x < width; x+=SPACING) {
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
		        for (unsigned int y = kin_hor_y; y < height; y++) {

		            if (isPixelGreen(img(x, y),LUT)) {
		                if (green_count == 0) {
		                    green_top = y;
		                }

		                green_count++;
		                // if VER_THRESHOLD green pixels found, add point
		                if (green_count == GREEN_HORIZON_MIN_GREEN_PIXELS) {//TODO
		                    arma::vec v(2);
		                    v[0] = x;
		                    v[1] = green_top;
		                    horizon_points.push_back(v);
		                    break;
		                }
		            } else {
		                // not green - reset
		                green_count = 0;
		            }
		        }
		    }

		    static int num_no_green = 0;
		    if(horizon_points.size() < 2) {
		        if(num_no_green < 150) {
		            num_no_green++;
		        } else {
		            num_no_green = 0;
		            log<NUClear::ERROR>("150 FRAMES OF NO GREEN HORIZON FOUND - VERY POOR LUT");
		        }

		        horizon_points.clear();
		        arma::vec v(2);
		        v[0] = 0;
		        v[1] = height-1;
		        horizon_points.push_back(v);
		        v[0] = width-1;
		        horizon_points.push_back(v);
		        
		        return horizon_points;
		    }

		    // statistical filter for green horizon points
		    double mean_y, std_dev_y;
		    arma::running_stat<double> acc;  //TODO

		    for(auto& p : horizon_points) {
		        if (p[1] < height-1)     // if not at bottom of image
		            acc(p[1]);
		    }

		    mean_y = acc.mean();
		    std_dev_y = acc.stddev();
		

		    std::vector<arma::vec>::iterator p = horizon_points.begin();

		    while(p < horizon_points.end()) {
		        if ((*p)[1] < mean_y - GREEN_HORIZON_UPPER_THRESHOLD_MULT * std_dev_y) {//TODO
		            thrown_points.push_back(*p);
		            p = horizon_points.erase(p);
		        } else {
		            p++;
		        }
		    }

		    //NOTE: OLD Code may have printed more info about the thrown points.
		    log<NUClear::DEBUG>("Green Horizon Thrown Points : ", thrown_points.size());

		    horizon_points = upperConvexHull(horizon_points);
		   
		    return horizon_points;
        }

        std::vector<arma::vec> GreenHorizon::upperConvexHull(const std::vector<arma::vec>& points) {
        	int n = points.size(), k = 0;
	        std::vector<arma::vec> H(n);

	        // Build upper hull
	        for (int i = 0; i < n; i++) {
	            while ((k >= 2) && (differenceCrossProduct2D(H[k-2], H[k-1], points[i]) <= 0))
	                k--;

	            H[k] = points[i];
	            k++;
	        }

	        H.resize(k);
	        return H;
	    }


        bool GreenHorizon::isPixelGreen(const messages::input::Image::Pixel& p, const LookUpTable& LUT) {
        	return LUT.classifyPixel(p) == green; //green is a Colour enum, defined in LookUpTable.h
        }

	}
}