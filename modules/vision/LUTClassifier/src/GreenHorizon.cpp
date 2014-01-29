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
        using messages::vision::Colour;
        using utility::vision::LookUpTable;
        
        GreenHorizon::GreenHorizon() {
        	// Empty constructor.
        }
        
        void GreenHorizon::setParameters(unsigned int GREEN_HORIZON_SCAN_SPACING_, unsigned int GREEN_HORIZON_MIN_GREEN_PIXELS_, float GREEN_HORIZON_UPPER_THRESHOLD_MULT_) {
            GREEN_HORIZON_SCAN_SPACING = GREEN_HORIZON_SCAN_SPACING_;
            GREEN_HORIZON_MIN_GREEN_PIXELS = GREEN_HORIZON_MIN_GREEN_PIXELS_;
            GREEN_HORIZON_UPPER_THRESHOLD_MULT = GREEN_HORIZON_UPPER_THRESHOLD_MULT_;
        }

        void GreenHorizon::calculateGreenHorizon(const Image& img, const LookUpTable& LUT) {
        	//NEEDS KINEMATICS ! const Horizon& kin_hor = Last<1,KinematicsHorizon>;

		    size_t width = img.width();
		    size_t height = img.height();
		    if(width==0 || height==0){
		    	NUClear::log<NUClear::ERROR>("Image height or width zero. Check camera!!");
		    	return;
		    }

		    //makes this fail-safe in the event of improper parameters
		    const int SPACING = std::max(GREEN_HORIZON_SCAN_SPACING, 1U);
		    
		    // variable declarations    
		    std::vector<arma::vec2> horizon_points;
		    horizon_points.reserve(1+width/SPACING);	//Reserve for optimisation
		    std::vector<arma::vec2> thrown_points;
			thrown_points.reserve(1+width/SPACING);	//Reserve for optimisation

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
		                    arma::vec2 v;
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
		    if (horizon_points.size() < 2) {
		        if (num_no_green < 150) {
		            num_no_green++;
		        }
				
				else {
		            num_no_green = 0;
                    NUClear::log<NUClear::WARN>("150 FRAMES OF NO GREEN HORIZON FOUND - VERY POOR LUT");
		        }

		        horizon_points.clear();
		        arma::vec2 v;
		        v[0] = 0;
		        v[1] = height - 1;
		        horizon_points.push_back(v);
		        v[0] = width - 1;
		        horizon_points.push_back(v);
		        set(horizon_points, width, height);		        
		    }

		    // statistical filter for green horizon points
		    double mean_y, std_dev_y;
		    arma::running_stat<double> acc;  //TODO

		    for(auto& p : horizon_points) {
		        if (p[1] < height - 1)     // if not at bottom of image
		            acc(p[1]);
		    }

		    mean_y = acc.mean();
		    std_dev_y = acc.stddev();

		    std::vector<arma::vec2>::iterator p = horizon_points.begin();

		    while(p < horizon_points.end()) {
		        if ((*p)[1] < mean_y - GREEN_HORIZON_UPPER_THRESHOLD_MULT * std_dev_y) {//TODO
		            thrown_points.push_back(*p);
		            p = horizon_points.erase(p);
		        }
				
				else {
		            p++;
		        }
		    }

		    //NOTE: OLD Code may have printed more info about the thrown points.
		    //std::cout << "Green Horizon Number of Thrown Points : " << thrown_points.size() << std::endl;

		    horizon_points = upperConvexHull(horizon_points);
            std::cout << horizon_points.size() << std::endl;
            for(auto& p : horizon_points){
                std::cout << "GreenHorizon - set point ( "<< p[0] << " , " << p[1]<<")."<< std::endl;
            }
           
            
		   	set(horizon_points, width, height);		    
        }

        void GreenHorizon::set(std::vector<arma::vec2> original_points, int image_width, int image_height) {

		    interpolated_points.clear();

		    //unsigned int position, y_new;
		    int y_new;
		    std::vector<arma::vec2>::const_iterator it_start, it_end;

		    //generate start/end edge points (if not there)
		    if (original_points.front()[0] > 0) {
		        double y = interpolate(original_points.at(0), original_points.at(1), 0);
				
		        //clamp to image vertical bounds
		        y = std::max(y, 0.0);
		        y = std::min(y, image_height - 1.0);
		        arma::vec2 v;
		        v[0] = 0;
		        v[1] = y;
		        original_points.insert(original_points.begin(), v);
		    }
			
		    if (original_points.back()[0] < image_width - 1) {
		        double y = interpolate(original_points.at(original_points.size() - 2),
		                               original_points.at(original_points.size() - 1),
		                               image_width - 1);
									   
		        //clamp to image vertical bounds
		        y = std::max(y, 0.0);
		        y = std::min(y, image_height - 1.0);
		        arma::vec2 v;
		        v[0] = image_width - 1;
		        v[1] = y;
		        original_points.push_back(v);
		    }

		    it_start = original_points.begin();
		    it_end = it_start + 1;
			
		    for (int x = 0; x < image_width; x++) {
		        // consider hull points either side of current x value
		        while (x > (*it_end)[0]) {
		            it_start++;
		            it_end++;
		        }
				//
		        // calculate y value for interpolated point
		        y_new = interpolate(*it_start, *it_end, x);

		        if(y_new >= image_height) {
		            std::cout << "GreenHorizon::set: " << y_new << " it_start: " << *it_start << " it_end: " << *it_end << std::endl;
				}
				
		        arma::vec2 v;
		        v[0] = x;
		        v[1] = y_new;
		        interpolated_points.push_back(v);
		    }
		}

        std::vector<arma::vec2> GreenHorizon::upperConvexHull(const std::vector<arma::vec2>& points) {
        	int n = points.size(), k = 0;
	        std::vector<arma::vec2> H(n);

	        // Build upper hull
	        for (int i = 0; i < n; i++) {
	            while ((k >= 2) && (differenceCrossProduct2D(H[k - 2], H[k - 1], points[i]) <= 0)) {
	                k--;
				}
				
	            H[k] = points[i];
	            k++;
	        }

	        H.resize(k);
	        return H;
	    }

        bool GreenHorizon::isPixelGreen(const messages::input::Image::Pixel& p, const LookUpTable& LUT) {
        	return LUT.classifyPixel(p) == Colour::green;
        }

		double GreenHorizon::interpolate(arma::vec2 p1, arma::vec2 p2, double x){
			return p1[1] + (p2[1] - p1[1]) * (x - p1[0]) / (p2[0] - p1[0]);
		}

		const std::vector<arma::vec2>& GreenHorizon::getInterpolatedPoints() const {
		    return interpolated_points;
		}

		std::vector<arma::vec2> GreenHorizon::getInterpolatedSubset(unsigned int spacing) const {
		    std::vector<arma::vec2> subset;
			
			spacing = std::max(spacing, 1U);
		    for (unsigned int i = 0; i < interpolated_points.size(); i += spacing) {
		        subset.push_back(interpolated_points.at(i));
		    }
			
		    return subset;
		}
	}
}
