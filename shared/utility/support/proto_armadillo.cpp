/*
 * This file is part of the Autocalibration Codebase.
 *
 * The Autocalibration Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The Autocalibration Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the Autocalibration Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2015 NUBots <nubots@nubots.net>
 */

#include "proto_armadillo.h"

template <int size>
struct Setter;

template <>
struct Setter<2> {

    template <typename Proto, typename Vector>
    static Proto& set_vector(Proto& proto, const Vector& vector) {
        proto.set_x(vector[0]);
        proto.set_y(vector[1]);
        return proto;
    }

    template <typename Proto, typename Matrix>
    static Proto& set_matrix(Proto& proto, const Matrix& matrix) {
        proto.mutable_x()->set_x(matrix(0,0));
        proto.mutable_x()->set_y(matrix(1,0));
        proto.mutable_y()->set_x(matrix(0,1));
        proto.mutable_y()->set_y(matrix(1,1));
        return proto;
    }
};

template <>
struct Setter<3> {

    template <typename Proto, typename Vector>
    static Proto& set_vector(Proto& proto, const Vector& vector) {
        proto.set_x(vector[0]);
        proto.set_y(vector[1]);
        proto.set_z(vector[2]);
        return proto;
    }

    template <typename Proto, typename Matrix>
    static Proto& set_matrix(Proto& proto, const Matrix& matrix) {
        proto.mutable_x()->set_x(matrix(0,0));
        proto.mutable_x()->set_y(matrix(1,0));
        proto.mutable_x()->set_z(matrix(2,0));
        proto.mutable_y()->set_x(matrix(0,1));
        proto.mutable_y()->set_y(matrix(1,1));
        proto.mutable_y()->set_z(matrix(2,1));
        proto.mutable_z()->set_x(matrix(0,2));
        proto.mutable_z()->set_y(matrix(1,2));
        proto.mutable_z()->set_z(matrix(2,2));
        return proto;
    }
};

template <>
struct Setter<4> {

    template <typename Proto, typename Vector>
    static Proto& set_vector(Proto& proto, const Vector& vector) {
        proto.set_x(vector[0]);
        proto.set_y(vector[1]);
        proto.set_z(vector[2]);
        proto.set_t(vector[3]);
        return proto;
    }

    template <typename Proto, typename Matrix>
    static Proto& set_matrix(Proto& proto, const Matrix& matrix) {
        proto.mutable_x()->set_x(matrix(0,0));
        proto.mutable_x()->set_y(matrix(1,0));
        proto.mutable_x()->set_z(matrix(2,0));
        proto.mutable_x()->set_t(matrix(3,0));
        proto.mutable_y()->set_x(matrix(0,1));
        proto.mutable_y()->set_y(matrix(1,1));
        proto.mutable_y()->set_z(matrix(2,1));
        proto.mutable_y()->set_t(matrix(3,1));
        proto.mutable_z()->set_x(matrix(0,2));
        proto.mutable_z()->set_y(matrix(1,2));
        proto.mutable_z()->set_z(matrix(2,2));
        proto.mutable_z()->set_t(matrix(3,2));
        proto.mutable_t()->set_x(matrix(0,3));
        proto.mutable_t()->set_y(matrix(1,3));
        proto.mutable_t()->set_z(matrix(2,3));
        proto.mutable_t()->set_t(matrix(3,3));
        return proto;
    }
};

message::vec2&  operator<< ( message::vec2& proto,  const arma::vec2& vec) {
    return Setter<2>::set_vector(proto, vec);
}
message::vec3&  operator<< ( message::vec3& proto,  const arma::vec3& vec) {
    return Setter<3>::set_vector(proto, vec);
}
message::vec4&  operator<< ( message::vec4& proto,  const arma::vec4& vec) {
    return Setter<4>::set_vector(proto, vec);
}
message::fvec2& operator<< (message::fvec2& proto, const arma::fvec2& vec) {
    return Setter<2>::set_vector(proto, vec);
}
message::fvec3& operator<< (message::fvec3& proto, const arma::fvec3& vec) {
    return Setter<3>::set_vector(proto, vec);
}
message::fvec4& operator<< (message::fvec4& proto, const arma::fvec4& vec) {
    return Setter<4>::set_vector(proto, vec);
}
message::ivec2& operator<< (message::ivec2& proto, const arma::ivec2& vec) {
    return Setter<2>::set_vector(proto, vec);
}
message::ivec3& operator<< (message::ivec3& proto, const arma::ivec3& vec) {
    return Setter<3>::set_vector(proto, vec);
}
message::ivec4& operator<< (message::ivec4& proto, const arma::ivec4& vec) {
    return Setter<4>::set_vector(proto, vec);
}
message::uvec2& operator<< (message::uvec2& proto, const arma::uvec2& vec) {
    return Setter<2>::set_vector(proto, vec);
}
message::uvec3& operator<< (message::uvec3& proto, const arma::uvec3& vec) {
    return Setter<3>::set_vector(proto, vec);
}
message::uvec4& operator<< (message::uvec4& proto, const arma::uvec4& vec) {
    return Setter<4>::set_vector(proto, vec);
}
message::vec2&  operator<< ( message::vec2& proto,  const arma::vec& vec) {
    return Setter<2>::set_vector(proto, vec);
}
message::vec3&  operator<< ( message::vec3& proto,  const arma::vec& vec) {
    return Setter<3>::set_vector(proto, vec);
}
message::vec4&  operator<< ( message::vec4& proto,  const arma::vec& vec) {
    return Setter<4>::set_vector(proto, vec);
}
message::fvec2& operator<< (message::fvec2& proto, const arma::fvec& vec) {
    return Setter<2>::set_vector(proto, vec);
}
message::fvec3& operator<< (message::fvec3& proto, const arma::fvec& vec) {
    return Setter<3>::set_vector(proto, vec);
}
message::fvec4& operator<< (message::fvec4& proto, const arma::fvec& vec) {
    return Setter<4>::set_vector(proto, vec);
}
message::ivec2& operator<< (message::ivec2& proto, const arma::ivec& vec) {
    return Setter<2>::set_vector(proto, vec);
}
message::ivec3& operator<< (message::ivec3& proto, const arma::ivec& vec) {
    return Setter<3>::set_vector(proto, vec);
}
message::ivec4& operator<< (message::ivec4& proto, const arma::ivec& vec) {
    return Setter<4>::set_vector(proto, vec);
}
message::uvec2& operator<< (message::uvec2& proto, const arma::uvec& vec) {
    return Setter<2>::set_vector(proto, vec);
}
message::uvec3& operator<< (message::uvec3& proto, const arma::uvec& vec) {
    return Setter<3>::set_vector(proto, vec);
}
message::uvec4& operator<< (message::uvec4& proto, const arma::uvec& vec) {
    return Setter<4>::set_vector(proto, vec);
}
message::mat22&  operator<< ( message::mat22& proto,  const arma::mat22& mat) {
    return Setter<2>::set_matrix(proto, mat);
}
message::mat33&  operator<< ( message::mat33& proto,  const arma::mat33& mat) {
    return Setter<3>::set_matrix(proto, mat);
}
message::mat44&  operator<< ( message::mat44& proto,  const arma::mat44& mat) {
    return Setter<4>::set_matrix(proto, mat);
}
message::fmat22& operator<< (message::fmat22& proto, const arma::fmat22& mat) {
    return Setter<2>::set_matrix(proto, mat);
}
message::fmat33& operator<< (message::fmat33& proto, const arma::fmat33& mat) {
    return Setter<3>::set_matrix(proto, mat);
}
message::fmat44& operator<< (message::fmat44& proto, const arma::fmat44& mat) {
    return Setter<4>::set_matrix(proto, mat);
}
message::imat22& operator<< (message::imat22& proto, const arma::imat22& mat) {
    return Setter<2>::set_matrix(proto, mat);
}
message::imat33& operator<< (message::imat33& proto, const arma::imat33& mat) {
    return Setter<3>::set_matrix(proto, mat);
}
message::imat44& operator<< (message::imat44& proto, const arma::imat44& mat) {
    return Setter<4>::set_matrix(proto, mat);
}
message::umat22& operator<< (message::umat22& proto, const arma::umat22& mat) {
    return Setter<2>::set_matrix(proto, mat);
}
message::umat33& operator<< (message::umat33& proto, const arma::umat33& mat) {
    return Setter<3>::set_matrix(proto, mat);
}
message::umat44& operator<< (message::umat44& proto, const arma::umat44& mat) {
    return Setter<4>::set_matrix(proto, mat);
}
 message::mat22& operator<< ( message::mat22& proto,  const arma::mat& mat) {
    return Setter<2>::set_matrix(proto, mat);
 }
 message::mat33& operator<< ( message::mat33& proto,  const arma::mat& mat) {
    return Setter<3>::set_matrix(proto, mat);
 }
 message::mat44& operator<< ( message::mat44& proto,  const arma::mat& mat) {
    return Setter<4>::set_matrix(proto, mat);
 }
message::fmat22& operator<< (message::fmat22& proto, const arma::fmat& mat) {
    return Setter<2>::set_matrix(proto, mat);
}
message::fmat33& operator<< (message::fmat33& proto, const arma::fmat& mat) {
    return Setter<3>::set_matrix(proto, mat);
}
message::fmat44& operator<< (message::fmat44& proto, const arma::fmat& mat) {
    return Setter<4>::set_matrix(proto, mat);
}
message::imat22& operator<< (message::imat22& proto, const arma::imat& mat) {
    return Setter<2>::set_matrix(proto, mat);
}
message::imat33& operator<< (message::imat33& proto, const arma::imat& mat) {
    return Setter<3>::set_matrix(proto, mat);
}
message::imat44& operator<< (message::imat44& proto, const arma::imat& mat) {
    return Setter<4>::set_matrix(proto, mat);
}
message::umat22& operator<< (message::umat22& proto, const arma::umat& mat) {
    return Setter<2>::set_matrix(proto, mat);
}
message::umat33& operator<< (message::umat33& proto, const arma::umat& mat) {
    return Setter<3>::set_matrix(proto, mat);
}
message::umat44& operator<< (message::umat44& proto, const arma::umat& mat) {
    return Setter<4>::set_matrix(proto, mat);
}
