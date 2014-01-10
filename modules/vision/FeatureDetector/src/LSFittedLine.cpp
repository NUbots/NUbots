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

#include "LSFittedLine.h"

namespace modules {
	namespace vision {
	
		LSFittedLine::LSFittedLine() {
			clearPoints();
		}


		LSFittedLine::LSFittedLine(const std::vector<arma::vec2>& pointList) {
			clearPoints();
			addPoints(pointList);
		}

		LSFittedLine::~LSFittedLine() {
			points.clear();
		}

		void LSFittedLine::clearPoints() {
			valid = false;
			sumX = 0;
			sumY = 0;
			sumX2 = 0;
			sumY2 = 0;
			sumXY = 0;
			MSD = 0;
			r2tls = 0;
			points.clear();
		}

		const std::vector<arma::vec2>& LSFittedLine::getPoints() const {
			return points;
		}

		void LSFittedLine::addPoint(const arma::vec2& point) {
			sumX += point[0];
			sumY += point[1];
			sumX2 += point[0] * point[0];
			sumY2 += point[1] * point[1];
			sumXY += point[0] * point[1];
			points.push_back(point);
			valid = (points.size() >= 2);
				
			if (valid) {
				calcLine();
			}
		}

		void LSFittedLine::addPoints(const std::vector<arma::vec2>& pointlist) {
			if (!pointlist.empty()) {
				for (const arma::vec2& point : pointlist) {
				    sumX += point[0];
				    sumY += point[1];
				    sumX2 += point[0] * point[0];
				    sumY2 += point[1] * point[1];
				    sumXY += point[0] * point[1];
				    
				    points.push_back(point);
				}
				
				valid = (points.size() >= 2);
				
				if (valid) {
				    calcLine();
				}
			}
		}

		void LSFittedLine::joinLine(const LSFittedLine& sourceLine) {
			sumX += sourceLine.sumX;
			sumY += sourceLine.sumY;
			sumX2 += sourceLine.sumX2;
			sumY2 += sourceLine.sumY2;
			sumXY += sourceLine.sumXY;

			for (const auto& point : sourceLine.points) {
				points.push_back(point);
			}
		
			valid = (points.size() >= 2);
				
			if (valid) {
				calcLine();
			}
		}

		arma::vec2 LSFittedLine::combinedR2TLSandMSD(const LSFittedLine &sourceLine) const {
			double sxx, syy, sxy, Sigma;
			double TsumX, TsumY, TsumX2, TsumY2, TsumXY, TnumPoints;
		
			TsumX = sumX + sourceLine.sumX;
			TsumY = sumY + sourceLine.sumY;
			TsumX2 = sumX2 + sourceLine.sumX2;
			TsumY2 = sumY2 + sourceLine.sumY2;
			TsumXY = sumXY + sourceLine.sumXY;
			TnumPoints = points.size() + sourceLine.points.size();
		
			arma::vec2 results;

			sxx = TsumX2 - ((TsumX * TsumX) / TnumPoints);
			syy = TsumY2 - ((TsumY * TsumY) / TnumPoints);
			sxy = TsumXY - ((TsumX * TsumY) / TnumPoints);
		
			Sigma = (sxx + syy - std::sqrt(((sxx - syy) * (sxx - syy)) + (4 * sxy * sxy))) / 2;
		
			results[0] = 1.0 - ((4.0 * Sigma * Sigma) / (((sxx + syy) * (sxx + syy)) + ((sxx - syy) * (sxx - syy)) + (4.0 * sxy * sxy)));
			results[1] = Sigma / TnumPoints;
		
			return results;
		}

		bool LSFittedLine::isValid() const {
			return valid;
		}
			
		size_t LSFittedLine::getNumPoints() const {
			return points.size();
		}
			
		double LSFittedLine::getMSD () const {
			return MSD;
		}

		double LSFittedLine::getr2tls () const {
			return r2tls;
		}

		void LSFittedLine::calcLine() {
			double sxx, syy, sxy, Sigma;
			double A = 0, B = 0, C = 0;					// Line: Ax + By = C ????
			unsigned int numPoints = points.size();

			sxx = sumX2 - ((sumX * sumX) / numPoints);
			syy = sumY2 - ((sumY * sumY) / numPoints);
			sxy = sumXY - ((sumX * sumY) / numPoints);
			Sigma = ((sxx + syy) - std::sqrt(((sxx - syy) * (sxx - syy)) + (4 * sxy * sxy))) / 2;
		
			MSD = Sigma / numPoints;
			r2tls = 1.0 - ((4.0 * Sigma * Sigma) / (((sxx + syy) * (sxx + syy)) + ((sxx - syy) * (sxx - syy)) + (4.0 * sxy * sxy)));


			if (sxx > syy) {
				A = -sxy;
				B = (sxx - Sigma);
				C = -((sumX * sxy) - ((sxx - Sigma) * sumY)) / numPoints;
			}
	
			else {
				A = (syy - Sigma);
				B = -sxy;
				C = -((sumY * sxy) - ((syy - Sigma) * sumX)) / numPoints;
			}
	
			setLine(A, B, C);
		}

		bool LSFittedLine::getEndPoints(arma::vec2& p1, arma::vec2& p2) const {
			if (points.size() < 2)
				return false;

			float min = std::numeric_limits<float>::max();
			float max = -std::numeric_limits<float>::max();
	
			std::vector< arma::vec2 >::const_iterator p, p_min, p_max;

		    for(p = points.begin(), p_min = p_max = p; p!=points.end(); p++) {
		        float trans_x = -m_B*(*p)[0] - m_A*(*p)[1];
		        if(trans_x < min) {
		            p_min = p;
		            min = trans_x;
		        }
		        else if(trans_x > max) {
		            p_max = p;
		            max = trans_x;
		        }
		    }
		    p1 = projectOnto(*p_min);
		    p2 = projectOnto(*p_max);
		    return true;
		}

		bool LSFittedLine::getOriginalEndPoints(arma::vec2& p1, arma::vec2& p2) const {
			if (points.size() < 2)
				return false;

			float min = std::numeric_limits<float>::max();
			float max = -std::numeric_limits<float>::max();

		    std::vector< arma::vec2 >::const_iterator p, p_min, p_max;

		    for(p = points.begin(), p_min = p_max = p; p!=points.end(); p++) {
		        float trans_x = -m_B*(*p)[0] - m_A*(*p)[1];
		        if(trans_x < min) {
		            p_min = p;
		            min = trans_x;
		        }
		        else if(trans_x > max) {
		            p_max = p;
		            max = trans_x;
		        }
		    }
		    p1 = *p_min;
		    p2 = *p_max;
		    return true;
		}

		double LSFittedLine::averageDistanceBetween(const LSFittedLine& other) const {
			if (valid && other.isValid()) {
				arma::vec2 ep1, ep2, other_ep1, other_ep2;

				// No need to check this works - line is only valid if there are at least 2 points.
				getEndPoints(ep1, ep2);
				other.getEndPoints(other_ep1, other_ep2);

				// Determine distances from the two possible pairings.
				double d1 = 0.5 * (arma::norm(ep1 - other_ep1, 2) + arma::norm(ep2 - other_ep2, 2));
				double d2 = 0.5 * (arma::norm(ep2 - other_ep1, 2) + arma::norm(ep1 - other_ep2, 2));
				       
				// Best pairing results in minimum distance.
				return std::min(d1, d2); 
			}
		
			// Test for this - distances should always be positive.
			return -1.0;
		}

	}
}
