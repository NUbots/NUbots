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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef UTILITY_SUPPORT_YAML_ARMADILLO_H
#define UTILITY_SUPPORT_YAML_ARMADILLO_H

#include <yaml-cpp/yaml.h>

#include <armadillo>
#include <iostream>

#include "utility/support/yaml_expression.h"

namespace YAML {

template <>
struct convert<arma::Col<int>> {
    static Node encode(const arma::Col<int>& rhs) {
        Node node;

        for (const double& d : rhs) {
            node.push_back(d);
        }

        return node;
    }

    static bool decode(const Node& node, arma::Col<int>& rhs) {
        rhs.resize(node.size());
        for (uint i = 0; i < node.size(); ++i) {
            rhs[i] = node[i].as<utility::support::Expression>();
        }

        return true;
    }
};

template <>
struct convert<arma::Col<int64_t>> {
    static Node encode(const arma::Col<int64_t>& rhs) {
        Node node;

        for (const double& d : rhs) {
            node.push_back(d);
        }

        return node;
    }

    static bool decode(const Node& node, arma::Col<int64_t>& rhs) {
        rhs.resize(node.size());
        for (uint i = 0; i < node.size(); ++i) {
            rhs[i] = node[i].as<utility::support::Expression>();
        }

        return true;
    }
};

template <>
struct convert<arma::vec> {
    static Node encode(const arma::vec& rhs) {
        Node node;

        for (const double& d : rhs) {
            node.push_back(d);
        }

        return node;
    }

    static bool decode(const Node& node, arma::vec& rhs) {
        rhs.resize(node.size());
        for (uint i = 0; i < node.size(); ++i) {
            rhs[i] = node[i].as<utility::support::Expression>();
        }

        return true;
    }
};

template <>
struct convert<arma::fvec> {
    static Node encode(const arma::fvec& rhs) {
        Node node;

        for (const float& d : rhs) {
            node.push_back(d);
        }

        return node;
    }

    static bool decode(const Node& node, arma::fvec& rhs) {
        rhs.resize(node.size());
        for (uint i = 0; i < node.size(); ++i) {
            rhs[i] = node[i].as<utility::support::Expression>();
        }

        return true;
    }
};

template <uint size>
struct convert<arma::vec::fixed<size>> {
    static Node encode(const arma::vec::fixed<size>& rhs) {
        Node node;

        for (uint i = 0; i < size; ++i) {
            node.push_back(rhs[i]);
        }

        return node;
    }

    static bool decode(const Node& node, arma::vec::fixed<size>& rhs) {
        if (node.size() == size) {

            for (uint i = 0; i < size; ++i) {
                rhs[i] = node[i].as<utility::support::Expression>();
            }

            return true;
        }
        else {
            return false;
        }
    }
};

template <>
struct convert<arma::mat> {
    // TODO: use arma::vec decoding for each row?
    static Node encode(const arma::mat& rhs) {
        Node node;

        for (uint i = 0; i < rhs.n_rows; ++i) {
            Node row;
            for (uint j = 0; j < rhs.n_cols; ++j) {
                row.push_back(rhs(i, j));
            }
            node.push_back(row);
        }

        return node;
    }

    static bool decode(const Node& node, arma::mat& rhs) {

        rhs.resize(node.size(), node[0].size());

        for (uint i = 0; i < node.size(); ++i) {
            for (uint j = 0; j < node[i].size(); ++j) {
                rhs(i, j) = node[i][j].as<utility::support::Expression>();
            }
        }

        return true;
    }
};

template <uint rows, uint cols>
struct convert<arma::mat::fixed<rows, cols>> {
    // TODO: use arma::vec decoding for each row?
    static Node encode(const arma::mat::fixed<rows, cols>& rhs) {
        Node node;

        for (uint i = 0; i < rows; ++i) {
            Node row;
            for (uint j = 0; j < cols; ++j) {
                row.push_back(rhs(i, j));
            }
            node.push_back(row);
        }

        return node;
    }

    static bool decode(const Node& node, arma::mat::fixed<rows, cols>& rhs) {
        if (node.size() == rows) {  // TODO: check cols

            for (uint i = 0; i < rows; ++i) {
                for (uint j = 0; j < cols; ++j) {
                    rhs(i, j) = node[i][j].as<utility::support::Expression>();
                }
            }

            return true;
        }
        else {
            return false;
        }
    }
};
}  // namespace YAML

#endif  // UTILITY_SUPPORT_yaml_armadillo_H
