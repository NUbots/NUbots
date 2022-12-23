#ifndef MESSAGE_CONVERSION_PROTO_NEUTRON_MAP_HPP
#define MESSAGE_CONVERSION_PROTO_NEUTRON_MAP_HPP

#include <chrono>
#include <google/protobuf/duration.pb.h>
#include <google/protobuf/timestamp.pb.h>
#include <nuclear_bits/clock.hpp>

#include "Matrix.pb.h"
#include "Vector.pb.h"
#include "math_types.hpp"

namespace message::conversion {
    /**
     * @brief This type maps vector types to their protocol buffer equivalents
     *
     * @tparam T the vector type to map
     */
    template <typename T>
    struct NeutronToProtobufMap;
    // clang-format off
    template <> struct NeutronToProtobufMap<::message::conversion::math::vec2>   { using type = ::vec2; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::fvec2>  { using type = ::fvec2; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::ivec2>  { using type = ::ivec2; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::uvec2>  { using type = ::uvec2; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::vec3>   { using type = ::vec3; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::fvec3>  { using type = ::fvec3; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::ivec3>  { using type = ::ivec3; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::uvec3>  { using type = ::uvec3; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::vec4>   { using type = ::vec4; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::fvec4>  { using type = ::fvec4; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::ivec4>  { using type = ::ivec4; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::uvec4>  { using type = ::uvec4; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::vec5>   { using type = ::vec5; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::fvec5>  { using type = ::fvec5; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::ivec5>  { using type = ::ivec5; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::uvec5>  { using type = ::uvec5; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::vec6>   { using type = ::vec6; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::fvec6>  { using type = ::fvec6; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::ivec6>  { using type = ::ivec6; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::uvec6>  { using type = ::uvec6; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::vec7>   { using type = ::vec7; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::fvec7>  { using type = ::fvec7; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::ivec7>  { using type = ::ivec7; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::uvec7>  { using type = ::uvec7; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::vec8>   { using type = ::vec8; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::fvec8>  { using type = ::fvec8; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::ivec8>  { using type = ::ivec8; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::uvec8>  { using type = ::uvec8; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::vec9>   { using type = ::vec9; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::fvec9>  { using type = ::fvec9; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::ivec9>  { using type = ::ivec9; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::uvec9>  { using type = ::uvec9; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::vec10>  { using type = ::vec10; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::fvec10> { using type = ::fvec10; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::ivec10> { using type = ::ivec10; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::uvec10> { using type = ::uvec10; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::vec11>  { using type = ::vec11; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::fvec11> { using type = ::fvec11; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::ivec11> { using type = ::ivec11; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::uvec11> { using type = ::uvec11; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::vec12>  { using type = ::vec12; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::fvec12> { using type = ::fvec12; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::ivec12> { using type = ::ivec12; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::uvec12> { using type = ::uvec12; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::vec13>  { using type = ::vec13; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::fvec13> { using type = ::fvec13; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::ivec13> { using type = ::ivec13; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::uvec13> { using type = ::uvec13; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::vec14>  { using type = ::vec14; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::fvec14> { using type = ::fvec14; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::ivec14> { using type = ::ivec14; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::uvec14> { using type = ::uvec14; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::vec15>  { using type = ::vec15; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::fvec15> { using type = ::fvec15; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::ivec15> { using type = ::ivec15; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::uvec15> { using type = ::uvec15; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::vec16>  { using type = ::vec16; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::fvec16> { using type = ::fvec16; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::ivec16> { using type = ::ivec16; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::uvec16> { using type = ::uvec16; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::vec>    { using type = ::vec; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::cvec>   { using type = ::cvec; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::fvec>   { using type = ::fvec; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::ivec>   { using type = ::ivec; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::uvec>   { using type = ::uvec; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::mat2>   { using type = ::mat2; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::fmat2>  { using type = ::fmat2; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::imat2>  { using type = ::imat2; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::umat2>  { using type = ::umat2; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::mat3>   { using type = ::mat3; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::fmat3>  { using type = ::fmat3; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::imat3>  { using type = ::imat3; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::umat3>  { using type = ::umat3; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::mat4>   { using type = ::mat4; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::fmat4>  { using type = ::fmat4; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::imat4>  { using type = ::imat4; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::umat4>  { using type = ::umat4; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::mat5>   { using type = ::mat5; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::fmat5>  { using type = ::fmat5; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::imat5>  { using type = ::imat5; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::umat5>  { using type = ::umat5; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::mat6>   { using type = ::mat6; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::fmat6>  { using type = ::fmat6; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::imat6>  { using type = ::imat6; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::umat6>  { using type = ::umat6; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::mat7>   { using type = ::mat7; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::fmat7>  { using type = ::fmat7; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::imat7>  { using type = ::imat7; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::umat7>  { using type = ::umat7; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::mat8>   { using type = ::mat8; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::fmat8>  { using type = ::fmat8; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::imat8>  { using type = ::imat8; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::umat8>  { using type = ::umat8; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::mat9>   { using type = ::mat9; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::fmat9>  { using type = ::fmat9; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::imat9>  { using type = ::imat9; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::umat9>  { using type = ::umat9; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::mat10>  { using type = ::mat10; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::fmat10> { using type = ::fmat10; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::imat10> { using type = ::imat10; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::umat10> { using type = ::umat10; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::mat11>  { using type = ::mat11; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::fmat11> { using type = ::fmat11; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::imat11> { using type = ::imat11; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::umat11> { using type = ::umat11; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::mat12>  { using type = ::mat12; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::fmat12> { using type = ::fmat12; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::imat12> { using type = ::imat12; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::umat12> { using type = ::umat12; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::mat13>  { using type = ::mat13; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::fmat13> { using type = ::fmat13; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::imat13> { using type = ::imat13; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::umat13> { using type = ::umat13; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::mat14>  { using type = ::mat14; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::fmat14> { using type = ::fmat14; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::imat14> { using type = ::imat14; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::umat14> { using type = ::umat14; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::mat15>  { using type = ::mat15; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::fmat15> { using type = ::fmat15; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::imat15> { using type = ::imat15; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::umat15> { using type = ::umat15; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::mat16>  { using type = ::mat16; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::fmat16> { using type = ::fmat16; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::imat16> { using type = ::imat16; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::umat16> { using type = ::umat16; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::mat>    { using type = ::mat; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::cmat>   { using type = ::cmat; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::fmat>   { using type = ::fmat; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::imat>   { using type = ::imat; };
    template <> struct NeutronToProtobufMap<::message::conversion::math::umat>   { using type = ::umat; };
    template <> struct NeutronToProtobufMap<NUClear::clock::time_point>  { using type = ::google::protobuf::Timestamp; };
    template <> struct NeutronToProtobufMap<NUClear::clock::duration>  { using type = ::google::protobuf::Duration; };
    // clang-format on

    template <typename T>
    using ProtoNeutron = typename NeutronToProtobufMap<T>::type;

}  // namespace message::conversion

#endif  // MESSAGE_CONVERSION_PROTO_NEUTRON_MAP_HPP
