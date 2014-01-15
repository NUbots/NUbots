/*
 * This file is part of GreenHorizon.
 *
 * GreenHorizon is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GreenHorizon is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GreenHorizon.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_VISION_GREENHORIZON_H
#define MODULES_VISION_GREENHORIZON_H

#include <nuclear> 
#include <string>
#include <armadillo>

#include "messages/input/Image.h"
#include "messages/support/Configuration.h"
#include "messages/vision/ClassifiedImage.h"

#include "LookUpTable.h"

namespace modules {
    namespace vision {

        /**
         * @brief Calculate green horzion.    
         *
         * @author Jake Fountain
         * @note Edited by Alex Biddulph
         */
        class GreenHorizon {
        private:
            unsigned int GREEN_HORIZON_SCAN_SPACING;
            unsigned int GREEN_HORIZON_MIN_GREEN_PIXELS;
            float GREEN_HORIZON_UPPER_THRESHOLD_MULT;
            std::vector<arma::vec2> original_points;      //! @variable The original hull points.
            std::vector<arma::vec2> interpolated_points;  //! @variable The interpolated points.
            
        public:
            GreenHorizon();
            
            /*! @brief Sets configured parameters for the green horizon.
            */
            void setParameters(unsigned int GREEN_HORIZON_SCAN_SPACING_, unsigned int GREEN_HORIZON_MIN_GREEN_PIXELS_, float GREEN_HORIZON_UPPER_THRESHOLD_MULT_);

        public:
            /*! @brief Computes the visual green horizon.
                Note that the use of kinematics horizon has been replaced by dummmy code 
                @param image The raw image
            */ 
            void calculateGreenHorizon(const messages::input::Image& image, const LookUpTable& LUT);
         
            /*! @brief Computes the green horizon characteristics
                @param initial_points the horizon points calculated by the calculateGreenHorizon method
            */ 
            void set(const std::vector<arma::vec2> &initial_points, int image_width, int image_height);

            /*! @brief Returns a std::list of points on the convex hull in counter-clockwise order.
             Note: the last point in the returned std::list is the same as the first one.
             */
            std::vector<arma::vec2> upperConvexHull(const std::vector<arma::vec2>& points);

            /*! @brief Returns a true if the specified pixel is coloured green.
             */
            bool isPixelGreen(const messages::input::Image::Pixel& p, const LookUpTable& LUT);


            /*! @brief  2D cross product of OA and OB std::vectors, i.e. z-component of their 3D cross product.
            Returns a positive value, if OAB makes a counter-clockwise turn,
            negative for clockwise turn, and zero if the points are collinear.
            */
            static double differenceCrossProduct2D(const arma::vec2& O, const arma::vec2& A, const arma::vec2& B) {
                return (A[0] - O[0]) * (B[1] - O[1]) - (A[1] - O[1]) * (B[0] - O[0]);
            }
            
            double interpolate(arma::vec2 p1, arma::vec2 p2, double x);
            
            //! Returns the interpolated points for the entire screen width.
            const std::vector<arma::vec2>& getInterpolatedPoints() const;
            
            //! Returns a list of interpolated points with a given spacing.
            std::vector<arma::vec2> getInterpolatedSubset(unsigned int spacing) const;
        };
    
    }  // vision
}  // modules

#endif  // MODULES_VISION_GREENHORIZON_H

