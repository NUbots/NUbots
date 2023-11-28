/*
 * MIT License
 *
 * Copyright (c) 2022 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef MESSAGE_CONVERSION_NEUTRON_TYPE_MAP_HPP
#define MESSAGE_CONVERSION_NEUTRON_TYPE_MAP_HPP

#include <chrono>
#include <google/protobuf/duration.pb.h>
#include <google/protobuf/timestamp.pb.h>
#include <nuclear_bits/clock.hpp>

#include "Matrix.pb.h"
#include "Transform.pb.h"
#include "Vector.pb.h"
#include "math_types.hpp"

namespace message::conversion {
    /**
     * @brief Classification types for use in classifying the different Neutron types
     */
    struct Chrono {};
    struct Vector {};
    struct Matrix {};
    struct DynamicVector {};
    struct DynamicMatrix {};
    struct Isometry {};
    struct Quaternion {};

    /**
     * @brief This type maps Neutron types to one of the above classification types
     *
     * @tparam T the Neutron type to map
     */
    template <typename T>
    struct NeutronTypeMap;
    // clang-format off
    template <> struct NeutronTypeMap<::message::conversion::math::vec2>   { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::fvec2>  { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::ivec2>  { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::uvec2>  { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::vec3>   { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::fvec3>  { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::ivec3>  { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::uvec3>  { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::vec4>   { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::fvec4>  { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::ivec4>  { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::uvec4>  { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::vec5>   { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::fvec5>  { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::ivec5>  { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::uvec5>  { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::vec6>   { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::fvec6>  { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::ivec6>  { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::uvec6>  { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::vec7>   { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::fvec7>  { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::ivec7>  { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::uvec7>  { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::vec8>   { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::fvec8>  { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::ivec8>  { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::uvec8>  { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::vec9>   { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::fvec9>  { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::ivec9>  { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::uvec9>  { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::vec10>  { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::fvec10> { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::ivec10> { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::uvec10> { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::vec11>  { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::fvec11> { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::ivec11> { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::uvec11> { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::vec12>  { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::fvec12> { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::ivec12> { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::uvec12> { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::vec13>  { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::fvec13> { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::ivec13> { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::uvec13> { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::vec14>  { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::fvec14> { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::ivec14> { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::uvec14> { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::vec15>  { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::fvec15> { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::ivec15> { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::uvec15> { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::vec16>  { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::fvec16> { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::ivec16> { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::uvec16> { using type = Vector; };
    template <> struct NeutronTypeMap<::message::conversion::math::vec>    { using type = DynamicVector; };
    template <> struct NeutronTypeMap<::message::conversion::math::cvec>   { using type = DynamicVector; };
    template <> struct NeutronTypeMap<::message::conversion::math::fvec>   { using type = DynamicVector; };
    template <> struct NeutronTypeMap<::message::conversion::math::ivec>   { using type = DynamicVector; };
    template <> struct NeutronTypeMap<::message::conversion::math::uvec>   { using type = DynamicVector; };
    template <> struct NeutronTypeMap<::message::conversion::math::mat2>   { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::fmat2>  { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::imat2>  { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::umat2>  { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::mat3>   { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::fmat3>  { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::imat3>  { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::umat3>  { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::mat4>   { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::fmat4>  { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::imat4>  { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::umat4>  { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::mat5>   { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::fmat5>  { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::imat5>  { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::umat5>  { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::mat6>   { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::fmat6>  { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::imat6>  { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::umat6>  { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::mat7>   { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::fmat7>  { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::imat7>  { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::umat7>  { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::mat8>   { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::fmat8>  { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::imat8>  { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::umat8>  { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::mat9>   { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::fmat9>  { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::imat9>  { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::umat9>  { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::mat10>  { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::fmat10> { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::imat10> { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::umat10> { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::mat11>  { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::fmat11> { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::imat11> { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::umat11> { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::mat12>  { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::fmat12> { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::imat12> { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::umat12> { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::mat13>  { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::fmat13> { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::imat13> { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::umat13> { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::mat14>  { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::fmat14> { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::imat14> { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::umat14> { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::mat15>  { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::fmat15> { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::imat15> { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::umat15> { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::mat16>  { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::fmat16> { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::imat16> { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::umat16> { using type = Matrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::mat>    { using type = DynamicMatrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::cmat>   { using type = DynamicMatrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::fmat>   { using type = DynamicMatrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::imat>   { using type = DynamicMatrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::umat>   { using type = DynamicMatrix; };
    template <> struct NeutronTypeMap<::message::conversion::math::iso2>   { using type = Isometry; };
    template <> struct NeutronTypeMap<::message::conversion::math::fiso2>  { using type = Isometry; };
    template <> struct NeutronTypeMap<::message::conversion::math::iso3>   { using type = Isometry; };
    template <> struct NeutronTypeMap<::message::conversion::math::fiso3>  { using type = Isometry; };
    template <> struct NeutronTypeMap<::message::conversion::math::quat>   { using type = Quaternion; };
    template <> struct NeutronTypeMap<::message::conversion::math::fquat>  { using type = Quaternion; };
    template <> struct NeutronTypeMap<NUClear::clock::time_point>          { using type = Chrono; };
    template <> struct NeutronTypeMap<NUClear::clock::duration>            { using type = Chrono; };
    // clang-format on

    template <typename T>
    using NeutronType = typename NeutronTypeMap<T>::type;


}  // namespace message::conversion

#endif  // MESSAGE_CONVERSION_NEUTRON_TYPE_MAP_HPP
