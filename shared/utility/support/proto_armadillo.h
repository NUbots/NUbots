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

#ifndef UTILITY_SUPPORT_PROTO_ARMADILLO_H
#define UTILITY_SUPPORT_PROTO_ARMADILLO_H

#include <armadillo>
#include "messages/Vector.pb.h"
#include "messages/Matrix.pb.h"

messages::vec2&  operator<< ( messages::vec2& proto,  const arma::vec2& vec);
messages::vec3&  operator<< ( messages::vec3& proto,  const arma::vec3& vec);
messages::vec4&  operator<< ( messages::vec4& proto,  const arma::vec4& vec);
messages::fvec2& operator<< (messages::fvec2& proto, const arma::fvec2& vec);
messages::fvec3& operator<< (messages::fvec3& proto, const arma::fvec3& vec);
messages::fvec4& operator<< (messages::fvec4& proto, const arma::fvec4& vec);
messages::ivec2& operator<< (messages::ivec2& proto, const arma::ivec2& vec);
messages::ivec3& operator<< (messages::ivec3& proto, const arma::ivec3& vec);
messages::ivec4& operator<< (messages::ivec4& proto, const arma::ivec4& vec);
messages::uvec2& operator<< (messages::uvec2& proto, const arma::uvec2& vec);
messages::uvec3& operator<< (messages::uvec3& proto, const arma::uvec3& vec);
messages::uvec4& operator<< (messages::uvec4& proto, const arma::uvec4& vec);

messages::vec2&  operator<< ( messages::vec2& proto,  const arma::vec& vec);
messages::vec3&  operator<< ( messages::vec3& proto,  const arma::vec& vec);
messages::vec4&  operator<< ( messages::vec4& proto,  const arma::vec& vec);
messages::fvec2& operator<< (messages::fvec2& proto, const arma::fvec& vec);
messages::fvec3& operator<< (messages::fvec3& proto, const arma::fvec& vec);
messages::fvec4& operator<< (messages::fvec4& proto, const arma::fvec& vec);
messages::ivec2& operator<< (messages::ivec2& proto, const arma::ivec& vec);
messages::ivec3& operator<< (messages::ivec3& proto, const arma::ivec& vec);
messages::ivec4& operator<< (messages::ivec4& proto, const arma::ivec& vec);
messages::uvec2& operator<< (messages::uvec2& proto, const arma::uvec& vec);
messages::uvec3& operator<< (messages::uvec3& proto, const arma::uvec& vec);
messages::uvec4& operator<< (messages::uvec4& proto, const arma::uvec& vec);

messages::mat22&  operator<< ( messages::mat22& proto,  const arma::mat22& mat);
messages::mat33&  operator<< ( messages::mat33& proto,  const arma::mat33& mat);
messages::mat44&  operator<< ( messages::mat44& proto,  const arma::mat44& mat);
messages::fmat22& operator<< (messages::fmat22& proto, const arma::fmat22& mat);
messages::fmat33& operator<< (messages::fmat33& proto, const arma::fmat33& mat);
messages::fmat44& operator<< (messages::fmat44& proto, const arma::fmat44& mat);
messages::imat22& operator<< (messages::imat22& proto, const arma::imat22& mat);
messages::imat33& operator<< (messages::imat33& proto, const arma::imat33& mat);
messages::imat44& operator<< (messages::imat44& proto, const arma::imat44& mat);
messages::umat22& operator<< (messages::umat22& proto, const arma::umat22& mat);
messages::umat33& operator<< (messages::umat33& proto, const arma::umat33& mat);
messages::umat44& operator<< (messages::umat44& proto, const arma::umat44& mat);

 messages::mat22& operator<< ( messages::mat22& proto,  const arma::mat& mat);
 messages::mat33& operator<< ( messages::mat33& proto,  const arma::mat& mat);
 messages::mat44& operator<< ( messages::mat44& proto,  const arma::mat& mat);
messages::fmat22& operator<< (messages::fmat22& proto, const arma::fmat& mat);
messages::fmat33& operator<< (messages::fmat33& proto, const arma::fmat& mat);
messages::fmat44& operator<< (messages::fmat44& proto, const arma::fmat& mat);
messages::imat22& operator<< (messages::imat22& proto, const arma::imat& mat);
messages::imat33& operator<< (messages::imat33& proto, const arma::imat& mat);
messages::imat44& operator<< (messages::imat44& proto, const arma::imat& mat);
messages::umat22& operator<< (messages::umat22& proto, const arma::umat& mat);
messages::umat33& operator<< (messages::umat33& proto, const arma::umat& mat);
messages::umat44& operator<< (messages::umat44& proto, const arma::umat& mat);

#endif
