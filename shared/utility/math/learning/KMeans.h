/* KMeans clustering with expectation maximisation refinement
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
		KMeans(){}
		struct KMeansConfig{
			//NOTE: values here are safe defaults; the values you will want depend on the problem domain
			int number_of_clusters = 1; // set the number of Gaussians
			int k_means_iterations = 1; // the number of iterations of the k-means algorithm
			int em_iterations = 0; // the number of iterations of the expectation maximisation algorithm
			float variance_floor = 1; // the variance floor (smallest allowed value) for the diagonal covariances
			bool print_status = false; // either true or false; enable or disable printing of progress during the k-means and EM algorithms
			//TODO:find the types of these
			std::string dist_mode = "eucl_dist"; // specifies the distance used during the seeding of initial means and k-means clustering:
			std::string seed_mode = "static_spread"; // specifies how the initial means are seeded prior to running k-means and/or EM algorithms

			/* Parameters for dist_mode:
			 * eucl_dist	   	Euclidean distance
			 * maha_dist	   	Mahalanobis distance, which uses a global diagonal covariance matrix estimated from the training samples
			 *
			 * Parameters for seed_mode:
			 * keep_existing	   	keep the existing model (do not modify the means, covariances and hefts)
			 * static_subset	   	a subset of the training samples (repeatable)
			 * random_subset	   	a subset of the training samples (random)
			 * static_spread	   	a maximally spread subset of training samples (repeatable)
			 * random_spread	   	a maximally spread subset of training samples (random start)
			 */
		};

		void configure(const YAML::Node& conf);
		bool learn(arma::mat data);
	private:
    	arma::gmm_diag clusterModel;
		KMeansConfig config;
	};
}
}
}







