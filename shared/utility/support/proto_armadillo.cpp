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

messages::vec2&  operator<< ( messages::vec2& proto,  const arma::vec2& vec) {
    return Setter<2>::set_vector(proto, vec);
}
messages::vec3&  operator<< ( messages::vec3& proto,  const arma::vec3& vec) {
    return Setter<3>::set_vector(proto, vec);
}
messages::vec4&  operator<< ( messages::vec4& proto,  const arma::vec4& vec) {
    return Setter<4>::set_vector(proto, vec);
}
messages::fvec2& operator<< (messages::fvec2& proto, const arma::fvec2& vec) {
    return Setter<2>::set_vector(proto, vec);
}
messages::fvec3& operator<< (messages::fvec3& proto, const arma::fvec3& vec) {
    return Setter<3>::set_vector(proto, vec);
}
messages::fvec4& operator<< (messages::fvec4& proto, const arma::fvec4& vec) {
    return Setter<4>::set_vector(proto, vec);
}
messages::ivec2& operator<< (messages::ivec2& proto, const arma::ivec2& vec) {
    return Setter<2>::set_vector(proto, vec);
}
messages::ivec3& operator<< (messages::ivec3& proto, const arma::ivec3& vec) {
    return Setter<3>::set_vector(proto, vec);
}
messages::ivec4& operator<< (messages::ivec4& proto, const arma::ivec4& vec) {
    return Setter<4>::set_vector(proto, vec);
}
messages::uvec2& operator<< (messages::uvec2& proto, const arma::uvec2& vec) {
    return Setter<2>::set_vector(proto, vec);
}
messages::uvec3& operator<< (messages::uvec3& proto, const arma::uvec3& vec) {
    return Setter<3>::set_vector(proto, vec);
}
messages::uvec4& operator<< (messages::uvec4& proto, const arma::uvec4& vec) {
    return Setter<4>::set_vector(proto, vec);
}
messages::vec2&  operator<< ( messages::vec2& proto,  const arma::vec& vec) {
    return Setter<2>::set_vector(proto, vec);
}
messages::vec3&  operator<< ( messages::vec3& proto,  const arma::vec& vec) {
    return Setter<3>::set_vector(proto, vec);
}
messages::vec4&  operator<< ( messages::vec4& proto,  const arma::vec& vec) {
    return Setter<4>::set_vector(proto, vec);
}
messages::fvec2& operator<< (messages::fvec2& proto, const arma::fvec& vec) {
    return Setter<2>::set_vector(proto, vec);
}
messages::fvec3& operator<< (messages::fvec3& proto, const arma::fvec& vec) {
    return Setter<3>::set_vector(proto, vec);
}
messages::fvec4& operator<< (messages::fvec4& proto, const arma::fvec& vec) {
    return Setter<4>::set_vector(proto, vec);
}
messages::ivec2& operator<< (messages::ivec2& proto, const arma::ivec& vec) {
    return Setter<2>::set_vector(proto, vec);
}
messages::ivec3& operator<< (messages::ivec3& proto, const arma::ivec& vec) {
    return Setter<3>::set_vector(proto, vec);
}
messages::ivec4& operator<< (messages::ivec4& proto, const arma::ivec& vec) {
    return Setter<4>::set_vector(proto, vec);
}
messages::uvec2& operator<< (messages::uvec2& proto, const arma::uvec& vec) {
    return Setter<2>::set_vector(proto, vec);
}
messages::uvec3& operator<< (messages::uvec3& proto, const arma::uvec& vec) {
    return Setter<3>::set_vector(proto, vec);
}
messages::uvec4& operator<< (messages::uvec4& proto, const arma::uvec& vec) {
    return Setter<4>::set_vector(proto, vec);
}
messages::mat22&  operator<< ( messages::mat22& proto,  const arma::mat22& mat) {
    return Setter<2>::set_matrix(proto, mat);
}
messages::mat33&  operator<< ( messages::mat33& proto,  const arma::mat33& mat) {
    return Setter<3>::set_matrix(proto, mat);
}
messages::mat44&  operator<< ( messages::mat44& proto,  const arma::mat44& mat) {
    return Setter<4>::set_matrix(proto, mat);
}
messages::fmat22& operator<< (messages::fmat22& proto, const arma::fmat22& mat) {
    return Setter<2>::set_matrix(proto, mat);
}
messages::fmat33& operator<< (messages::fmat33& proto, const arma::fmat33& mat) {
    return Setter<3>::set_matrix(proto, mat);
}
messages::fmat44& operator<< (messages::fmat44& proto, const arma::fmat44& mat) {
    return Setter<4>::set_matrix(proto, mat);
}
messages::imat22& operator<< (messages::imat22& proto, const arma::imat22& mat) {
    return Setter<2>::set_matrix(proto, mat);
}
messages::imat33& operator<< (messages::imat33& proto, const arma::imat33& mat) {
    return Setter<3>::set_matrix(proto, mat);
}
messages::imat44& operator<< (messages::imat44& proto, const arma::imat44& mat) {
    return Setter<4>::set_matrix(proto, mat);
}
messages::umat22& operator<< (messages::umat22& proto, const arma::umat22& mat) {
    return Setter<2>::set_matrix(proto, mat);
}
messages::umat33& operator<< (messages::umat33& proto, const arma::umat33& mat) {
    return Setter<3>::set_matrix(proto, mat);
}
messages::umat44& operator<< (messages::umat44& proto, const arma::umat44& mat) {
    return Setter<4>::set_matrix(proto, mat);
}
 messages::mat22& operator<< ( messages::mat22& proto,  const arma::mat& mat) {
    return Setter<2>::set_matrix(proto, mat);
 }
 messages::mat33& operator<< ( messages::mat33& proto,  const arma::mat& mat) {
    return Setter<3>::set_matrix(proto, mat);
 }
 messages::mat44& operator<< ( messages::mat44& proto,  const arma::mat& mat) {
    return Setter<4>::set_matrix(proto, mat);
 }
messages::fmat22& operator<< (messages::fmat22& proto, const arma::fmat& mat) {
    return Setter<2>::set_matrix(proto, mat);
}
messages::fmat33& operator<< (messages::fmat33& proto, const arma::fmat& mat) {
    return Setter<3>::set_matrix(proto, mat);
}
messages::fmat44& operator<< (messages::fmat44& proto, const arma::fmat& mat) {
    return Setter<4>::set_matrix(proto, mat);
}
messages::imat22& operator<< (messages::imat22& proto, const arma::imat& mat) {
    return Setter<2>::set_matrix(proto, mat);
}
messages::imat33& operator<< (messages::imat33& proto, const arma::imat& mat) {
    return Setter<3>::set_matrix(proto, mat);
}
messages::imat44& operator<< (messages::imat44& proto, const arma::imat& mat) {
    return Setter<4>::set_matrix(proto, mat);
}
messages::umat22& operator<< (messages::umat22& proto, const arma::umat& mat) {
    return Setter<2>::set_matrix(proto, mat);
}
messages::umat33& operator<< (messages::umat33& proto, const arma::umat& mat) {
    return Setter<3>::set_matrix(proto, mat);
}
messages::umat44& operator<< (messages::umat44& proto, const arma::umat& mat) {
    return Setter<4>::set_matrix(proto, mat);
}

messages::vec& operator<< (messages::vec& proto, const arma::vec& vec) {
    for(uint i = 0; i < vec.n_elem; ++i) {
        proto.add_v(vec[i]);
    }
    return proto;
}
messages::fvec& operator<< (messages::fvec& proto, const arma::fvec& vec) {
    for(uint i = 0; i < vec.n_elem; ++i) {
        proto.add_v(vec[i]);
    }
    return proto;
}
messages::ivec& operator<< (messages::ivec& proto, const arma::ivec& vec) {
    for(uint i = 0; i < vec.n_elem; ++i) {
        proto.add_v(vec[i]);
    }
    return proto;
}
messages::uvec& operator<< (messages::uvec& proto, const arma::uvec& vec) {
    for(uint i = 0; i < vec.n_elem; ++i) {
        proto.add_v(vec[i]);
    }
    return proto;
}

messages::mat& operator<< ( messages::mat& proto, const arma::mat& mat) {
    for(uint x = 0; x < mat.n_cols; ++x) {
        auto& col = *proto.add_v();
        for (uint y = 0; y < mat.n_rows; ++y) {
            col.add_v(mat(x, y));
        }
    }
    return proto;
}
messages::fmat& operator<< (messages::fmat& proto, const arma::fmat& mat) {
    for(uint x = 0; x < mat.n_cols; ++x) {
        auto& col = *proto.add_v();
        for (uint y = 0; y < mat.n_rows; ++y) {
            col.add_v(mat(x, y));
        }
    }
    return proto;
}
messages::imat& operator<< (messages::imat& proto, const arma::imat& mat) {
    for(uint x = 0; x < mat.n_cols; ++x) {
        auto& col = *proto.add_v();
        for (uint y = 0; y < mat.n_rows; ++y) {
            col.add_v(mat(x, y));
        }
    }
    return proto;
}
messages::umat& operator<< (messages::umat& proto, const arma::umat& mat) {
    for(uint x = 0; x < mat.n_cols; ++x) {
        auto& col = *proto.add_v();
        for (uint y = 0; y < mat.n_rows; ++y) {
            col.add_v(mat(x, y));
        }
    }
    return proto;
}
