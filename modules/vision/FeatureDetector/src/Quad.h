/*
 * This file is part of NUBots FeatureDetector.
 *
 * NUBots FeatureDetector is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * NUBots FeatureDetector is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with NUBots FeatureDetector.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_VISION_QUAD_H
#define MODULES_VISION_QUAD_H

#include <nuclear>
#include <armadillo>
#include <ostream>
#include <vector>

namespace modules {
	namespace vision {
	
		class Quad {
		public:		
			Quad();
			Quad(const Quad& other);
			Quad(const arma::vec2& bottomLeft, const arma::vec2& topLeft, const arma::vec2& topRight, const arma::vec2& bottomRight);
			Quad(int left, int top, int right, int bottom);

			/**
			  * Sets the Quad as a screen aligned rectangle given the specified positions.
			  * @param left     The left x pixel value.
			  * @param top      The top y pixel value.
			  * @param right    The right x pixel value.
			  * @param bottom   The bottom y pixel value.
			  */
			void set(int left, int top, int right, int bottom);

			/**
			  * Sets the Quad given the specified corners.
			  * @param bottomLeft  The bottom left corner.
			  * @param topLeft     The top left corner.
			  * @param topRight    The top right corner.
			  * @param bottomRight The bottom right corner.
			  */
			void set(const arma::vec2& bottomLeft, const arma::vec2& topLeft, const arma::vec2& topRight, const arma::vec2& bottomRight);
		
			arma::vec2 getTopCentre() const;								//! Returns the bottom centre pixel location of the Quad.
			arma::vec2 getBottomCentre() const;								//! Returns the bottom centre pixel location of the Quad.

			arma::vec2 getCentre() const;									//! Returns the centre pixel location  of the Quad.

			arma::vec2 getBottomLeft() const;								//! Returns the bottom left pixel location  of the Quad.
			arma::vec2 getBottomRight() const;								//! Returns the bottom right pixel location  of the Quad.
			arma::vec2 getTopLeft() const;									//! Returns the top left pixel location  of the Quad.
			arma::vec2 getTopRight() const;									//! Returns the top right pixel location  of the Quad.

			double getLeft() const;
			double getRight() const;
			double getTop() const;
			double getBottom() const;

			int getBaseWidth() const;										//! Returns the base width of the Quad in pixels.
			int getTopWidth() const;										//! Returns the top width of the Quad in pixels.

			int getLeftHeight() const;										//! Returns the left height of the Quad in pixels.
			int getRightHeight() const;										//! Returns the right height of the Quad in pixels.

			double getAverageWidth() const;									//! Returns the average width of the Quad in pixels.
			double getAverageHeight() const;								//! Returns the average height of the Quad in pixels.

			double area() const;
			double aspectRatio() const;

			std::vector<arma::vec2> getVertices();

			bool overlapsHorizontally(const Quad& other) const;

		private:
			arma::vec2 bl;													//! @variable The bottom-left of the Quad.
			arma::vec2 br;													//! @variable The bottom-right of the Quad.
			arma::vec2 tr;													//! @variable The top-right of the Quad.
			arma::vec2 tl;													//! @variable The top-left of the Quad.

			//! @brief output stream operator.
		friend std::ostream& operator<< (std::ostream& output, const Quad& quad);
		
		//! @brief output stream operator for a vector of goals.
		friend std::ostream& operator<< (std::ostream& output, const std::vector<Quad>& quads);
		};
		
		

	}
}

#endif // MODULES_VISION_QUAD_H

