/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_BEHAVIOUR_SKILLS_SEARCHER_H
#define MODULES_BEHAVIOUR_SKILLS_SEARCHER_H

#include <nuclear>
#include <armadillo>


namespace module {
    namespace behaviour{
        namespace skills {

            /**
             * Executes a HeadBehaviourSoccer action.
             *
             * @author Jake Fountain

             Template argument T must be a normed linear space using arma::norm();
             */
            template<class T>
            class Searcher {

			private:

				std::vector<T> points;
				int current;
				T refPoint;

				bool new_goal;

				NUClear::clock::time_point lastSwitchTime;
				float switch_period;

				bool forward = true;

            public:
	           	Searcher():current(0), new_goal(false), switch_period(1000.0f){
					lastSwitchTime = NUClear::clock::now();
					//Init points to something sane
					points = std::vector<T>(1,arma::vec({0,0}));
				}

				~Searcher(){}

        		void sort(){
        			//Just set to closest:
        			auto relativePoints = points;
        			for (auto& p : relativePoints){
        				p = p - refPoint;
        			}
	        		auto iter = std::min_element(relativePoints.begin(),relativePoints.end(),comparator);
	        		// current = std::distance(relativePoints.begin(), iter);
					current = current % int(points.size());

	        		//Full sort
	        		// std::vector<std::pair<int, T>> relativePoints;
        			// for (uint i = 0; i < points.size(); i++){
        			// 	relativePoints.push_back(std::pair<int, T>(i,points[i] - refPoint));
        			// }
        			// std::sort(relativePoints.begin(),relativePoints.end(),pair_comparator);
        			// std::vector<T> newPoints(relativePoints.size());
        			// for(uint i = 0; i < relativePoints.size(); i++){
        			// 	newPoints[i] = points[relativePoints[i].first];
        			// }
        			// points = newPoints;
	        	}

	        	int size(){
	        		return points.size();
	        	}

        		static bool pair_comparator(const std::pair<int, T>& a, const std::pair<int, T>& b){
	        		return arma::norm(a.second) <  arma::norm(b.second);
	        	}

				static bool comparator(const T& a, const T& b){
	        		return arma::norm(a) <  arma::norm(b);
	        	}

        		void setSwitchTime(float dt){
	        		switch_period = dt;
	        	}

				void update(bool oscillate){
					//TODO: recode this garbage
					auto now = NUClear::clock::now();
					if(!new_goal && std::chrono::duration_cast<std::chrono::milliseconds>(now - lastSwitchTime).count() > switch_period){
						new_goal = true;
						if(forward){
							int new_index = current + 1;
							if(oscillate){
								//Moves search forward and backward along path
								if (new_index >= points.size()){
									forward = false;
									current = std::max(current - 1, 0);
								} else {
									current = new_index;
								}
							} else {
								//Loops path
								current = new_index % int(points.size());
							}
						} else {
							int new_index = current - 1;
							if(oscillate){
								//Moves search forward and backward along path
								if (new_index < 0){
									forward = true;
									current = std::min(int(points.size())-1, 1);
								} else {
									current = new_index;
								}
							} else {
								//Loops path
								current = new_index % int(points.size());
							}
						}
						lastSwitchTime = now;
					}
				}

				bool newGoal(){
					return new_goal;
				}

				T getState(){
					new_goal = false;
					return points[current % points.size()];
				}

				void replaceSearchPoints(const std::vector<T>& ps, const T& refPoint_){
					refPoint = refPoint_;
					points = ps;
					sort();
					new_goal = true;
				}

				void translate(const T& delta){
					for(auto& p : points){
						p += delta;
					}
					new_goal = true;
				}

            };

        }  // motion
    } //behaviour
}  // modules

#endif  // MODULES_BEHAVIOURS_SKILLS_SEARCHER_H

