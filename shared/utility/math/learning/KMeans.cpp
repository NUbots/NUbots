/* 
 * This file is part of Learning utilities.
 *
 * Learning utilities is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
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
#include "KMeans.h"

#include "utility/support/yaml_expression.h"

namespace utility {
namespace math {
namespace learning {

	void KMeans::configure(const YAML::Node& conf){
		number_of_clusters = config["number_of_clusters"].as<int>();
		k_means_iterations = config["k_means_iterations"].as<int>();
		em_iterations = config["em_iterations"].as<int>();

		variance_floor = config["variance_floor"].as<Expression>();

		eucl_dist = config["eucl_dist"].as<>();
		random_spread = config["random_spread"].as<>();
		print_mode = config["print_mode"].as<>();
	}


    arma::gmm_diag clusterModel;
    clusterModel.learn(data, number_of_clusters, arma::eucl_dist, arma::random_spread, k_means_iterations, em_iterations, variance_floor, print_mode);
    clusterModel.learn(data, number_of_clusters, arma::maha_dist, arma::random_spread, k_means_iterations, em_iterations, variance_floor, print_mode);

}
}
}







