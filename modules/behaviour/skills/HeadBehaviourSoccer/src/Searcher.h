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


namespace modules {
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

            public:
	           	Searcher():current(0), new_goal(false), switch_period(1000.0f){
					lastSwitchTime = NUClear::clock::now();
					//Init points to something sane
					points = std::vector<T>(1,arma::vec({0,0}));
				}

				~Searcher(){}
        		
        		void sort(){
        			auto relativePoints = points;
        			for (auto& p : relativePoints){
        				p = p - refPoint;
        			}
	        		auto iter = std::min_element(relativePoints.begin(),relativePoints.end(),comparator);
	        		current = std::distance(relativePoints.begin(), iter);
	        	}

        		static bool comparator(const T& a, const T& b){
	        		return arma::norm(a) <  arma::norm(b);
	        	}

        		void setSwitchTime(float dt){
	        		switch_period = dt;
	        	}

				void update(){
					auto now = NUClear::clock::now();
					if(!new_goal && std::chrono::duration_cast<std::chrono::milliseconds>(now - lastSwitchTime).count() > switch_period){
						new_goal = true;
						current = (current + 1) % points.size();
						lastSwitchTime = now;
					}
				}

				bool newGoal(){
					return new_goal;
				}

				T getState(){
					new_goal = false;
					std::cout << "Getting state[" << current % points.size() << "] : " << points[current % points.size()] << std::endl;
					return points[current % points.size()];
				}

				void replaceSearchPoints(const std::vector<T>& ps, const T& refPoint_){
					refPoint = refPoint_;
					points = ps;
					sort();
					new_goal = true;
				}

            };

        }  // motion
    } //behaviour
}  // modules

#endif  // MODULES_BEHAVIOURS_SKILLS_SEARCHER_H

