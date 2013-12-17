/*
 * This file is part of FeatureDetector.
 *
 * FeatureDetector is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * FeatureDetector is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with FeatureDetector.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_VISION_LSFITTEDLINE_H
#define MODULES_VISION_LSFITTEDLINE_H

#include <armadillo>
#include <vector>

#include "Line.h"

namespace modules {
	namespace vision {
	
		class LSFittedLine : public Line {
		public:
			LSFittedLine();
			LSFittedLine(const std::vector<arma::vec2>& pointlist);
			virtual ~LSFittedLine();
			
			void addPoint(const arma::vec2& point);
			void addPoints(const std::vector<arma::vec2>& pointlist);
			void clearPoints();
			size_t getNumPoints() const;
			const std::vector<arma::vec2>& getPoints() const;
			bool getEndPoints(arma::vec2& p1, arma::vec2& p2) const;
			bool getOriginalEndPoints(arma::vec2& p1, arma::vec2& p2) const;
			
			void joinLine(const LSFittedLine& sourceLine);
			
			arma::vec2 combinedR2TLSandMSD(const LSFittedLine& sourceLine) const;
			
			double getMSD() const;
			double getr2tls() const;
			
			double averageDistanceBetween(const LSFittedLine& other) const;

			bool isValid() const;
			
		private:
			void calcLine();
			
			double sumX, sumY, sumX2, sumY2, sumXY;
			double MSD, r2tls;
			std::vector<arma::vec2> points;
			
			bool valid;
		};

	}
}

#endif // MODULES_VISION_LSFITTEDLINE_H
