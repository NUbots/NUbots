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

            /************************************
             *      FUNCTION PROTOTYPES         *
             ************************************/
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

            /************************************
             *      FUNCTION IMPLEMENTATIONS    *
             ************************************/
            template<class Model, typename DataPoint>
            std::vector<std::pair<Model, std::vector<DataPoint>>> findMultipleModels(const std::vector<DataPoint>& points, 
                                                    double e, 
                                                    unsigned int n, 
                                                    unsigned int k, 
                                                    unsigned int max_iterations, 
                                                    RANSAC_SELECTION_METHOD method) {
                double variance;
                bool found;
                Model model;
                std::vector<std::pair<Model, std::vector<DataPoint>>> results;
                std::vector<DataPoint> consensus;
                std::vector<DataPoint> remainder;

                // Run first iterations.
                found = findModel(points, model, consensus, remainder, variance, e, n, k, method);
                
                if (found) {
                    results.push_back(std::pair<Model, std::vector<DataPoint>>(model, consensus));
                    
                    while (found && results.size() < max_iterations) {
                        found = findModel(remainder, model, consensus, remainder, variance, e, n, k, method);
                        
                        if(found)
                            results.push_back(std::pair<Model, std::vector<DataPoint>>(model, consensus));
                    }
                }

                return results;
            }

            template<class Model, typename DataPoint>
            bool findModel(std::vector<DataPoint> points, 
                            Model &result, 
                            std::vector<DataPoint>& consensus, 
                            std::vector<DataPoint>& remainder, 
                            double& variance, 
                            double e, 
                            unsigned int n, 
                            unsigned int k, 
                            RANSAC_SELECTION_METHOD method) {
                if ((points.size() < n) || (n < result.minPointsForFit())) {
                    return false;
                }

                // Used for BestFittingConsensus method.
                double minerr = std::numeric_limits<double>::max();
                
                // Used for LargestConsensus method.
                size_t largestconsensus = 0;
                
                // Arrays for storing concensus sets.
                bool c1[points.size()];
                bool c2[points.size()];

                bool* best_concensus = c1;
                bool* cur_concensus;
                double cur_variance;
                bool found = false;

                for (unsigned int i = 0; i < k; ++i) {
                    // Randomly select 2 distinct points.
                    Model m = generateRandomModel<Model, DataPoint>(points);
                    cur_variance = 0;

                    unsigned int concensus_size = 0;

                    // Use the concensus that is not currently the best.
                    cur_concensus = ((c1 == best_concensus) ? c2 : c1);   

                    // Determine consensus set.
                    for (size_t i = 0; i < points.size(); i++) {
                        double dist = m.calculateError(points.at(i));
                        
                        // Cheap and nasty - assuming that true.
                        bool in = (dist < e);
                        cur_variance += (dist * in);
                        concensus_size += in;
                        cur_concensus[i] = in;
                        
        //                if (dist < e) {
        //                    cur_variance += dist;
        //                    concensus_size++;
        //                    cur_concensus[i] = true;
        //                }
        //                else {
        //                    cur_concensus[i] = false;
        //                }
                    }

                    // Normalise the variance.
                    cur_variance /= concensus_size;

                    // Determine whether the consensus is better.
                    if(concensus_size >= n) {
                        switch(method) {
                            case LargestConsensus: {
                                if(concensus_size > largestconsensus) {
                                    found = true;
                                    result = m;
                                    largestconsensus = concensus_size;
                                    minerr = cur_variance;							// Keep variance for other purposes.
                                    best_concensus = cur_concensus;
                                }
                                
                                break;
                            }
                            
                            case BestFittingConsensus: {
                                if(cur_variance < minerr) {
                                    found = true;
                                    result = m;
                                    minerr = cur_variance;
                                    best_concensus = cur_concensus;
                                }
                                
                                break;
                            }
                        }
                    }
                }
                
                variance = minerr;

                if(found) {
                    consensus.clear();
                    remainder.clear();

                    for(unsigned int i = 0; i < points.size(); i++) {
                        if(best_concensus[i]) {
                            consensus.push_back(points.at(i));
                        }
                        
                        else {
                            remainder.push_back(points.at(i));
                        }
                    }
                }
                
                return found;
            }

            //NOTE: Assumes that there are no duplicates in the data
            //      point set (only way to guarantee no infinite loop)
            template<class Model, typename DataPoint>
            Model generateRandomModel(const std::vector<DataPoint>& points) {
                Model model;
                size_t n = points.size();
                
                if (n >= model.minPointsForFit()) {
                    std::vector<size_t> indices;
                    size_t next;
                    
                    indices.push_back(rand() % n);

                    while(indices.size() < model.minPointsForFit()) {
                        bool unique;
                        
                        do {
                            unique = true;
                            next = rand() % n;
                            
                            for (size_t i : indices) {
                                if(i == next)
                                    unique = false;
                            }		                
                        } while(!unique);
                        
                        indices.push_back(next);
                    }

                    std::vector<DataPoint> rand_pts;
                    
                    for (size_t i : indices) {
                        rand_pts.push_back(points.at(i));
                    }

                    model.regenerate(rand_pts);
                }
                
                return model;
        //        Model model;
        //        if(points.size() >= model.minPointsForFit()) {
        //            std::vector<DataPoint> rand_pts;
        //            DataPoint next;
        //            rand_pts.push_back(points.at(rand() % points.size()));

        //            while(rand_pts.size() < model.minPointsForFit()) {
        //                bool unique;
        //                do {
        //                    unique = true;
        //                    next = points.at(rand() % points.size());
        //                    BOOST_FOREACH(DataPoint& pt, rand_pts) {
        //                        if(pt == next)
        //                            unique = false;
        //                    }
        //                }
        //                while(!unique);
        //                rand_pts.push_back(next);
        //            }
        //            model.regenerate(rand_pts);
        //        }
        //        return model;

		}

	}
}

#endif //  MODULES_VISION_RANSAC_H
