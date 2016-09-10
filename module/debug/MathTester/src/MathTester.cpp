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

#include "MathTester.h"
#include "message/support/Configuration.h"
#include "utility/support/yaml_armadillo.h"
#include "utility/math/geometry/Polygon.h"

namespace module {
    namespace debug {
        using utility::math::geometry::Polygon;
        using message::support::Configuration;

        MathTester::MathTester(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
            on<Configuration>("MathTester.yaml").then([this](const Configuration& tests) {

                testPolygon(true, tests.config);
                // if(tests["test_polygon"]){
                //     bool polygonPass = true;
                //     try{
                //         polygonPass = testPolygon(false,tests.config);
                //     } catch (const std::exception& e){
                //         testPolygon(true,tests.config);
                //     }
                //     if(!polygonPass){
                //         testPolygon(true,tests.config);
                //     }
                // }
            });
        }

        bool MathTester::testPolygon(bool verbose, const YAML::Node&){

            if(verbose) {}
            std::cerr << "Beginning polygon test.===============" << std::endl;
            std::vector<arma::vec2> vertices;//config["polygon_vertices"].as<std::vector<arma::vec2>>()
            // for(auto& v : config["polygon_vertices"].as<std::vector<std::vector<double>>>()){
            //     vertices.push_back(arma::vec2{v[0],v[1]});
            // }
            vertices.push_back(arma::vec2{0,1.5});
            vertices.push_back(arma::vec2{-1,-1});
            vertices.push_back(arma::vec2{0,0});
            vertices.push_back(arma::vec2{1,-1});
            Polygon poly(vertices);
            std::cerr << "Generated polygon with vertices:" << std::endl;
            for(auto& v : vertices){
                std::cerr << v.t();
            }


            std::cerr << "Creating sample points:" << std::endl;
            std::vector<arma::vec2> samples;
            int number_of_samples = 50;//config["number_of_samples"].as<int>()
            float r = 1.2;//config["sample_radius"].as<double>();
            for(int i = 0; i < number_of_samples; i++){
                float theta = 2 * M_PI * i / float(number_of_samples) ;
                arma::vec2 p = {r * std::cos(theta), r * std::sin(theta)};
                samples.push_back(p);
                std::cerr << i << " " << p.t();
            }


            //Test point contained
            std::cerr << "Testing containment:" << std::endl;
            int i = 0;
            for(auto& sample : samples){
                std::cerr << i++ << " " << poly.pointContained(sample) << std::endl;
            }


            //Test projectPointToPolygon
            std::cerr << "Testing projection:" << std::endl;
            i = 0;
            for(auto& sample : samples){
                std::cerr << i++ << " " << poly.projectPointToPolygon(sample).t();
            }
            return true;
        }

    } // debug
} // modules
