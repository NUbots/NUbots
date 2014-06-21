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

#ifndef UTILITY_SUPPORT_ARMAYAMLCONVERSION_H
#define UTILITY_SUPPORT_ARMAYAMLCONVERSION_H

#include <armadillo>
#include <iostream> 
#include <yaml-cpp/yaml.h>


namespace YAML {

    template<>
    struct convert<arma::vec> {
        static Node encode(const arma::vec& rhs) {
            Node node;

            for(const double& d : rhs) {
                node.push_back(d);
            }

            return node;
        }

        static bool decode(const Node& node, arma::vec& rhs) {
            rhs.resize(node.size());
            for (uint i = 0; i < node.size(); ++i) {
                rhs[i] = node[i].as<double>();
            }

            return true;
        }
    };

    template<uint size>
    struct convert<arma::vec::fixed<size>> {
        static Node encode(const arma::vec::fixed<size>& rhs) {
            Node node;

            for(uint i = 0; i < size; ++i) {
                node.push_back(rhs[i]);
            }

            return node;
        }

        static bool decode(const Node& node, arma::vec::fixed<size>& rhs) {
            if(node.size() == size) {

                for(uint i = 0; i < size; ++i) {
                    rhs[i] = node[i].as<double>();
                }

                return true;
            }
            else {
                return false;
            }
        }
    };
}

#endif