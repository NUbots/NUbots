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

#ifndef MODULES_VISION_RANSAC_H
#define MODULES_VISION_RANSAC_H

#include <nuclear>
#include <utility>
#include <vector>

namespace modules {
	namespace vision {
	
			enum RANSAC_SELECTION_METHOD {
				LargestConsensus,
				BestFittingConsensus
			};

			//Model must provide several features
			template<class Model, typename DataPoint>
			std::vector<std::pair<Model, std::vector<DataPoint>>> findMultipleModels(const std::vector<DataPoint>& line_points,
				                                                        double e,
				                                                        unsigned int n,
				                                                        unsigned int k,
				                                                        unsigned int max_iterations,
				                                                        RANSAC_SELECTION_METHOD method);

			template<class Model, typename DataPoint>
			bool findModel(std::vector<DataPoint> points,
				           Model& result,
				           std::vector<DataPoint>& consensus,
				           std::vector<DataPoint>& remainder,
				           double& variance,
				           double e,
				           unsigned int n,
				           unsigned int k,
				           RANSAC_SELECTION_METHOD method);

			template<class Model, typename DataPoint>
			Model generateRandomModel(const std::vector<DataPoint>& points);
		}

		#include "ransac.template"
	}
}

#endif //  MODULES_VISION_RANSAC_H
