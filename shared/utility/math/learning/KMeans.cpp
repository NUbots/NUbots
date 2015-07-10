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
		config.number_of_clusters = conf["number_of_clusters"].as<int>();
		config.k_means_iterations = conf["k_means_iterations"].as<int>();
		config.em_iterations = conf["em_iterations"].as<int>();
		config.variance_floor = conf["variance_floor"].as<utility::support::Expression>();
		config.print_status = conf["print_status"].as<bool>();

		config.dist_mode = conf["dist_mode"].as<std::string>();
		config.seed_mode = conf["seed_mode"].as<std::string>();
	}

	bool KMeans::learn(arma::mat data){
		if(config.dist_mode == "maha_dist") {
			auto distance_mode = arma::maha_dist;
			if(config.seed_mode == "keep_existing"){
				auto seed_mode_arma = arma::keep_existing;
   				return clusterModel.learn(data, config.number_of_clusters, distance_mode, seed_mode_arma, config.k_means_iterations, config.em_iterations, config.variance_floor, config.print_status);
			}else if(config.seed_mode == "static_subset"){
				auto seed_mode_arma = arma::static_subset;
   				return clusterModel.learn(data, config.number_of_clusters, distance_mode, seed_mode_arma, config.k_means_iterations, config.em_iterations, config.variance_floor, config.print_status);
			}else if(config.seed_mode == "random_subset"){
				auto seed_mode_arma = arma::random_subset;
   				return clusterModel.learn(data, config.number_of_clusters, distance_mode, seed_mode_arma, config.k_means_iterations, config.em_iterations, config.variance_floor, config.print_status);
			}else if(config.seed_mode == "static_spread"){
				auto seed_mode_arma = arma::static_spread;
   				return clusterModel.learn(data, config.number_of_clusters, distance_mode, seed_mode_arma, config.k_means_iterations, config.em_iterations, config.variance_floor, config.print_status);
			}else if(config.seed_mode == "random_spread"){
				auto seed_mode_arma = arma::random_spread;
   				return clusterModel.learn(data, config.number_of_clusters, distance_mode, seed_mode_arma, config.k_means_iterations, config.em_iterations, config.variance_floor, config.print_status);
			} else {
				auto seed_mode_arma = arma::static_spread;
   				return clusterModel.learn(data, config.number_of_clusters, distance_mode, seed_mode_arma, config.k_means_iterations, config.em_iterations, config.variance_floor, config.print_status);
			}
		} else {
			auto distance_mode = arma::eucl_dist;
			if(config.seed_mode == "keep_existing"){
				auto seed_mode_arma = arma::keep_existing;
   				return clusterModel.learn(data, config.number_of_clusters, distance_mode, seed_mode_arma, config.k_means_iterations, config.em_iterations, config.variance_floor, config.print_status);
			}else if(config.seed_mode == "static_subset"){
				auto seed_mode_arma = arma::static_subset;
   				return clusterModel.learn(data, config.number_of_clusters, distance_mode, seed_mode_arma, config.k_means_iterations, config.em_iterations, config.variance_floor, config.print_status);
			}else if(config.seed_mode == "random_subset"){
				auto seed_mode_arma = arma::random_subset;
   				return clusterModel.learn(data, config.number_of_clusters, distance_mode, seed_mode_arma, config.k_means_iterations, config.em_iterations, config.variance_floor, config.print_status);
			}else if(config.seed_mode == "static_spread"){
				auto seed_mode_arma = arma::static_spread;
   				return clusterModel.learn(data, config.number_of_clusters, distance_mode, seed_mode_arma, config.k_means_iterations, config.em_iterations, config.variance_floor, config.print_status);
			}else if(config.seed_mode == "random_spread"){
				auto seed_mode_arma = arma::random_spread;
   				return clusterModel.learn(data, config.number_of_clusters, distance_mode, seed_mode_arma, config.k_means_iterations, config.em_iterations, config.variance_floor, config.print_status);
			} else {
				auto seed_mode_arma = arma::static_spread;
   				return clusterModel.learn(data, config.number_of_clusters, distance_mode, seed_mode_arma, config.k_means_iterations, config.em_iterations, config.variance_floor, config.print_status);
			}
		}
	}

}
}
}







