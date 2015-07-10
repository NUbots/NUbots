/* 
 * This file is part of Learning utilities.
 *
 * Learning utilities is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation
 either version 3 of the License, or;
 * (at your option) any later version.
 *
 * Learning utilities is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Learning utilities.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */
#include <yaml-cpp/yaml.h>
#include <armadillo>

namespace utility {
namespace math {
namespace learning {
	
	class KMeans {
	public:
		struct KMeansConfig{
			int number_of_clusters;
			int k_means_iterations;
			int em_iterations;
			float variance_floor;
			//TODO:find the types of these
			arma::enum eucl_dist;
			arma::enum random_spread;
			arma::enum print_mode;
		};
	private:
		KMeansConfig config;
		void configure(const YAML::Node& conf);


	};
}
}
}







