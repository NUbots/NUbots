#ifndef MESSAGE_CONVERSION_PROTO_MATRIX_H
#define MESSAGE_CONVERSION_PROTO_MATRIX_H

#include "Matrix.pb.h"
#include "Vector.pb.h"
#include "math_types.h"

namespace message {
namespace conversion {

    /**
     * @brief This type maps protocol buffer types to their vector equivalents
     *
     * @tparam T the protocol buffer type to map
     */
    template <typename T>
    struct ProtobufToVectorMap;
    // clang-format off
    template <> struct ProtobufToVectorMap<::vec2>   { using type = ::message::conversion::math::vec2; };
    template <> struct ProtobufToVectorMap<::fvec2>  { using type = ::message::conversion::math::fvec2; };
    template <> struct ProtobufToVectorMap<::ivec2>  { using type = ::message::conversion::math::ivec2; };
    template <> struct ProtobufToVectorMap<::uvec2>  { using type = ::message::conversion::math::uvec2; };
    template <> struct ProtobufToVectorMap<::vec3>   { using type = ::message::conversion::math::vec3; };
    template <> struct ProtobufToVectorMap<::fvec3>  { using type = ::message::conversion::math::fvec3; };
    template <> struct ProtobufToVectorMap<::ivec3>  { using type = ::message::conversion::math::ivec3; };
    template <> struct ProtobufToVectorMap<::uvec3>  { using type = ::message::conversion::math::uvec3; };
    template <> struct ProtobufToVectorMap<::vec4>   { using type = ::message::conversion::math::vec4; };
    template <> struct ProtobufToVectorMap<::fvec4>  { using type = ::message::conversion::math::fvec4; };
    template <> struct ProtobufToVectorMap<::ivec4>  { using type = ::message::conversion::math::ivec4; };
    template <> struct ProtobufToVectorMap<::uvec4>  { using type = ::message::conversion::math::uvec4; };
    template <> struct ProtobufToVectorMap<::vec5>   { using type = ::message::conversion::math::vec5; };
    template <> struct ProtobufToVectorMap<::fvec5>  { using type = ::message::conversion::math::fvec5; };
    template <> struct ProtobufToVectorMap<::ivec5>  { using type = ::message::conversion::math::ivec5; };
    template <> struct ProtobufToVectorMap<::uvec5>  { using type = ::message::conversion::math::uvec5; };
    template <> struct ProtobufToVectorMap<::vec6>   { using type = ::message::conversion::math::vec6; };
    template <> struct ProtobufToVectorMap<::fvec6>  { using type = ::message::conversion::math::fvec6; };
    template <> struct ProtobufToVectorMap<::ivec6>  { using type = ::message::conversion::math::ivec6; };
    template <> struct ProtobufToVectorMap<::uvec6>  { using type = ::message::conversion::math::uvec6; };
    template <> struct ProtobufToVectorMap<::vec7>   { using type = ::message::conversion::math::vec7; };
    template <> struct ProtobufToVectorMap<::fvec7>  { using type = ::message::conversion::math::fvec7; };
    template <> struct ProtobufToVectorMap<::ivec7>  { using type = ::message::conversion::math::ivec7; };
    template <> struct ProtobufToVectorMap<::uvec7>  { using type = ::message::conversion::math::uvec7; };
    template <> struct ProtobufToVectorMap<::vec8>   { using type = ::message::conversion::math::vec8; };
    template <> struct ProtobufToVectorMap<::fvec8>  { using type = ::message::conversion::math::fvec8; };
    template <> struct ProtobufToVectorMap<::ivec8>  { using type = ::message::conversion::math::ivec8; };
    template <> struct ProtobufToVectorMap<::uvec8>  { using type = ::message::conversion::math::uvec8; };
    template <> struct ProtobufToVectorMap<::vec9>   { using type = ::message::conversion::math::vec9; };
    template <> struct ProtobufToVectorMap<::fvec9>  { using type = ::message::conversion::math::fvec9; };
    template <> struct ProtobufToVectorMap<::ivec9>  { using type = ::message::conversion::math::ivec9; };
    template <> struct ProtobufToVectorMap<::uvec9>  { using type = ::message::conversion::math::uvec9; };
    template <> struct ProtobufToVectorMap<::vec10>  { using type = ::message::conversion::math::vec10; };
    template <> struct ProtobufToVectorMap<::fvec10> { using type = ::message::conversion::math::fvec10; };
    template <> struct ProtobufToVectorMap<::ivec10> { using type = ::message::conversion::math::ivec10; };
    template <> struct ProtobufToVectorMap<::uvec10> { using type = ::message::conversion::math::uvec10; };
    template <> struct ProtobufToVectorMap<::vec11>  { using type = ::message::conversion::math::vec11; };
    template <> struct ProtobufToVectorMap<::fvec11> { using type = ::message::conversion::math::fvec11; };
    template <> struct ProtobufToVectorMap<::ivec11> { using type = ::message::conversion::math::ivec11; };
    template <> struct ProtobufToVectorMap<::uvec11> { using type = ::message::conversion::math::uvec11; };
    template <> struct ProtobufToVectorMap<::vec12>  { using type = ::message::conversion::math::vec12; };
    template <> struct ProtobufToVectorMap<::fvec12> { using type = ::message::conversion::math::fvec12; };
    template <> struct ProtobufToVectorMap<::ivec12> { using type = ::message::conversion::math::ivec12; };
    template <> struct ProtobufToVectorMap<::uvec12> { using type = ::message::conversion::math::uvec12; };
    template <> struct ProtobufToVectorMap<::vec13>  { using type = ::message::conversion::math::vec13; };
    template <> struct ProtobufToVectorMap<::fvec13> { using type = ::message::conversion::math::fvec13; };
    template <> struct ProtobufToVectorMap<::ivec13> { using type = ::message::conversion::math::ivec13; };
    template <> struct ProtobufToVectorMap<::uvec13> { using type = ::message::conversion::math::uvec13; };
    template <> struct ProtobufToVectorMap<::vec14>  { using type = ::message::conversion::math::vec14; };
    template <> struct ProtobufToVectorMap<::fvec14> { using type = ::message::conversion::math::fvec14; };
    template <> struct ProtobufToVectorMap<::ivec14> { using type = ::message::conversion::math::ivec14; };
    template <> struct ProtobufToVectorMap<::uvec14> { using type = ::message::conversion::math::uvec14; };
    template <> struct ProtobufToVectorMap<::vec15>  { using type = ::message::conversion::math::vec15; };
    template <> struct ProtobufToVectorMap<::fvec15> { using type = ::message::conversion::math::fvec15; };
    template <> struct ProtobufToVectorMap<::ivec15> { using type = ::message::conversion::math::ivec15; };
    template <> struct ProtobufToVectorMap<::uvec15> { using type = ::message::conversion::math::uvec15; };
    template <> struct ProtobufToVectorMap<::vec16>  { using type = ::message::conversion::math::vec16; };
    template <> struct ProtobufToVectorMap<::fvec16> { using type = ::message::conversion::math::fvec16; };
    template <> struct ProtobufToVectorMap<::ivec16> { using type = ::message::conversion::math::ivec16; };
    template <> struct ProtobufToVectorMap<::uvec16> { using type = ::message::conversion::math::uvec16; };
    // clang-format on
    template <typename T>
    using VecProto = typename ProtobufToVectorMap<T>::type;

    template <typename T>
    struct DynamicProtobufToVectorMap;
    // clang-format off
    template <> struct DynamicProtobufToVectorMap<::vec>  { using type = ::message::conversion::math::vec; };
    template <> struct DynamicProtobufToVectorMap<::fvec> { using type = ::message::conversion::math::fvec; };
    template <> struct DynamicProtobufToVectorMap<::ivec> { using type = ::message::conversion::math::ivec; };
    template <> struct DynamicProtobufToVectorMap<::uvec> { using type = ::message::conversion::math::uvec; };
    // clang-format on
    template <typename T>
    using DynamicVecProto = typename DynamicProtobufToVectorMap<T>::type;

    /**
     * @brief This type maps vector types to their protocol buffer equivalents
     *
     * @tparam T the vector type to map
     */
    template <typename T>
    struct VectorToProtobufMap;
    // clang-format off
    template <> struct VectorToProtobufMap<::message::conversion::math::vec2>   { using type = ::vec2; };
    template <> struct VectorToProtobufMap<::message::conversion::math::fvec2>  { using type = ::fvec2; };
    template <> struct VectorToProtobufMap<::message::conversion::math::ivec2>  { using type = ::ivec2; };
    template <> struct VectorToProtobufMap<::message::conversion::math::uvec2>  { using type = ::uvec2; };
    template <> struct VectorToProtobufMap<::message::conversion::math::vec3>   { using type = ::vec3; };
    template <> struct VectorToProtobufMap<::message::conversion::math::fvec3>  { using type = ::fvec3; };
    template <> struct VectorToProtobufMap<::message::conversion::math::ivec3>  { using type = ::ivec3; };
    template <> struct VectorToProtobufMap<::message::conversion::math::uvec3>  { using type = ::uvec3; };
    template <> struct VectorToProtobufMap<::message::conversion::math::vec4>   { using type = ::vec4; };
    template <> struct VectorToProtobufMap<::message::conversion::math::fvec4>  { using type = ::fvec4; };
    template <> struct VectorToProtobufMap<::message::conversion::math::ivec4>  { using type = ::ivec4; };
    template <> struct VectorToProtobufMap<::message::conversion::math::uvec4>  { using type = ::uvec4; };
    template <> struct VectorToProtobufMap<::message::conversion::math::vec5>   { using type = ::vec5; };
    template <> struct VectorToProtobufMap<::message::conversion::math::fvec5>  { using type = ::fvec5; };
    template <> struct VectorToProtobufMap<::message::conversion::math::ivec5>  { using type = ::ivec5; };
    template <> struct VectorToProtobufMap<::message::conversion::math::uvec5>  { using type = ::uvec5; };
    template <> struct VectorToProtobufMap<::message::conversion::math::vec6>   { using type = ::vec6; };
    template <> struct VectorToProtobufMap<::message::conversion::math::fvec6>  { using type = ::fvec6; };
    template <> struct VectorToProtobufMap<::message::conversion::math::ivec6>  { using type = ::ivec6; };
    template <> struct VectorToProtobufMap<::message::conversion::math::uvec6>  { using type = ::uvec6; };
    template <> struct VectorToProtobufMap<::message::conversion::math::vec7>   { using type = ::vec7; };
    template <> struct VectorToProtobufMap<::message::conversion::math::fvec7>  { using type = ::fvec7; };
    template <> struct VectorToProtobufMap<::message::conversion::math::ivec7>  { using type = ::ivec7; };
    template <> struct VectorToProtobufMap<::message::conversion::math::uvec7>  { using type = ::uvec7; };
    template <> struct VectorToProtobufMap<::message::conversion::math::vec8>   { using type = ::vec8; };
    template <> struct VectorToProtobufMap<::message::conversion::math::fvec8>  { using type = ::fvec8; };
    template <> struct VectorToProtobufMap<::message::conversion::math::ivec8>  { using type = ::ivec8; };
    template <> struct VectorToProtobufMap<::message::conversion::math::uvec8>  { using type = ::uvec8; };
    template <> struct VectorToProtobufMap<::message::conversion::math::vec9>   { using type = ::vec9; };
    template <> struct VectorToProtobufMap<::message::conversion::math::fvec9>  { using type = ::fvec9; };
    template <> struct VectorToProtobufMap<::message::conversion::math::ivec9>  { using type = ::ivec9; };
    template <> struct VectorToProtobufMap<::message::conversion::math::uvec9>  { using type = ::uvec9; };
    template <> struct VectorToProtobufMap<::message::conversion::math::vec10>  { using type = ::vec10; };
    template <> struct VectorToProtobufMap<::message::conversion::math::fvec10> { using type = ::fvec10; };
    template <> struct VectorToProtobufMap<::message::conversion::math::ivec10> { using type = ::ivec10; };
    template <> struct VectorToProtobufMap<::message::conversion::math::uvec10> { using type = ::uvec10; };
    template <> struct VectorToProtobufMap<::message::conversion::math::vec11>  { using type = ::vec11; };
    template <> struct VectorToProtobufMap<::message::conversion::math::fvec11> { using type = ::fvec11; };
    template <> struct VectorToProtobufMap<::message::conversion::math::ivec11> { using type = ::ivec11; };
    template <> struct VectorToProtobufMap<::message::conversion::math::uvec11> { using type = ::uvec11; };
    template <> struct VectorToProtobufMap<::message::conversion::math::vec12>  { using type = ::vec12; };
    template <> struct VectorToProtobufMap<::message::conversion::math::fvec12> { using type = ::fvec12; };
    template <> struct VectorToProtobufMap<::message::conversion::math::ivec12> { using type = ::ivec12; };
    template <> struct VectorToProtobufMap<::message::conversion::math::uvec12> { using type = ::uvec12; };
    template <> struct VectorToProtobufMap<::message::conversion::math::vec13>  { using type = ::vec13; };
    template <> struct VectorToProtobufMap<::message::conversion::math::fvec13> { using type = ::fvec13; };
    template <> struct VectorToProtobufMap<::message::conversion::math::ivec13> { using type = ::ivec13; };
    template <> struct VectorToProtobufMap<::message::conversion::math::uvec13> { using type = ::uvec13; };
    template <> struct VectorToProtobufMap<::message::conversion::math::vec14>  { using type = ::vec14; };
    template <> struct VectorToProtobufMap<::message::conversion::math::fvec14> { using type = ::fvec14; };
    template <> struct VectorToProtobufMap<::message::conversion::math::ivec14> { using type = ::ivec14; };
    template <> struct VectorToProtobufMap<::message::conversion::math::uvec14> { using type = ::uvec14; };
    template <> struct VectorToProtobufMap<::message::conversion::math::vec15>  { using type = ::vec15; };
    template <> struct VectorToProtobufMap<::message::conversion::math::fvec15> { using type = ::fvec15; };
    template <> struct VectorToProtobufMap<::message::conversion::math::ivec15> { using type = ::ivec15; };
    template <> struct VectorToProtobufMap<::message::conversion::math::uvec15> { using type = ::uvec15; };
    template <> struct VectorToProtobufMap<::message::conversion::math::vec16>  { using type = ::vec16; };
    template <> struct VectorToProtobufMap<::message::conversion::math::fvec16> { using type = ::fvec16; };
    template <> struct VectorToProtobufMap<::message::conversion::math::ivec16> { using type = ::ivec16; };
    template <> struct VectorToProtobufMap<::message::conversion::math::uvec16> { using type = ::uvec16; };
    // clang-format on
    template <typename T>
    using ProtoVec = typename VectorToProtobufMap<T>::type;

    template <typename T>
    struct DynamicVectorToProtobufMap;
    // clang-format off
    template <> struct DynamicVectorToProtobufMap<::message::conversion::math::vec>  { using type = ::vec; };
    template <> struct DynamicVectorToProtobufMap<::message::conversion::math::fvec> { using type = ::fvec; };
    template <> struct DynamicVectorToProtobufMap<::message::conversion::math::ivec> { using type = ::ivec; };
    template <> struct DynamicVectorToProtobufMap<::message::conversion::math::uvec> { using type = ::uvec; };
    // clang-format on
    template <typename T>
    using DynamicProtoVec = typename DynamicVectorToProtobufMap<T>::type;

    /**
     * @brief This type maps protocol buffer types to their matrix equivalents
     *
     * @tparam T the protocol buffer type to map
     */
    template <typename T>
    struct ProtobufToMatrixMap;
    // clang-format off
    template <> struct ProtobufToMatrixMap<::mat2>   { using type = ::message::conversion::math::mat2; };
    template <> struct ProtobufToMatrixMap<::fmat2>  { using type = ::message::conversion::math::fmat2; };
    template <> struct ProtobufToMatrixMap<::imat2>  { using type = ::message::conversion::math::imat2; };
    template <> struct ProtobufToMatrixMap<::umat2>  { using type = ::message::conversion::math::umat2; };
    template <> struct ProtobufToMatrixMap<::mat3>   { using type = ::message::conversion::math::mat3; };
    template <> struct ProtobufToMatrixMap<::fmat3>  { using type = ::message::conversion::math::fmat3; };
    template <> struct ProtobufToMatrixMap<::imat3>  { using type = ::message::conversion::math::imat3; };
    template <> struct ProtobufToMatrixMap<::umat3>  { using type = ::message::conversion::math::umat3; };
    template <> struct ProtobufToMatrixMap<::mat4>   { using type = ::message::conversion::math::mat4; };
    template <> struct ProtobufToMatrixMap<::fmat4>  { using type = ::message::conversion::math::fmat4; };
    template <> struct ProtobufToMatrixMap<::imat4>  { using type = ::message::conversion::math::imat4; };
    template <> struct ProtobufToMatrixMap<::umat4>  { using type = ::message::conversion::math::umat4; };
    template <> struct ProtobufToMatrixMap<::mat5>   { using type = ::message::conversion::math::mat5; };
    template <> struct ProtobufToMatrixMap<::fmat5>  { using type = ::message::conversion::math::fmat5; };
    template <> struct ProtobufToMatrixMap<::imat5>  { using type = ::message::conversion::math::imat5; };
    template <> struct ProtobufToMatrixMap<::umat5>  { using type = ::message::conversion::math::umat5; };
    template <> struct ProtobufToMatrixMap<::mat6>   { using type = ::message::conversion::math::mat6; };
    template <> struct ProtobufToMatrixMap<::fmat6>  { using type = ::message::conversion::math::fmat6; };
    template <> struct ProtobufToMatrixMap<::imat6>  { using type = ::message::conversion::math::imat6; };
    template <> struct ProtobufToMatrixMap<::umat6>  { using type = ::message::conversion::math::umat6; };
    template <> struct ProtobufToMatrixMap<::mat7>   { using type = ::message::conversion::math::mat7; };
    template <> struct ProtobufToMatrixMap<::fmat7>  { using type = ::message::conversion::math::fmat7; };
    template <> struct ProtobufToMatrixMap<::imat7>  { using type = ::message::conversion::math::imat7; };
    template <> struct ProtobufToMatrixMap<::umat7>  { using type = ::message::conversion::math::umat7; };
    template <> struct ProtobufToMatrixMap<::mat8>   { using type = ::message::conversion::math::mat8; };
    template <> struct ProtobufToMatrixMap<::fmat8>  { using type = ::message::conversion::math::fmat8; };
    template <> struct ProtobufToMatrixMap<::imat8>  { using type = ::message::conversion::math::imat8; };
    template <> struct ProtobufToMatrixMap<::umat8>  { using type = ::message::conversion::math::umat8; };
    template <> struct ProtobufToMatrixMap<::mat9>   { using type = ::message::conversion::math::mat9; };
    template <> struct ProtobufToMatrixMap<::fmat9>  { using type = ::message::conversion::math::fmat9; };
    template <> struct ProtobufToMatrixMap<::imat9>  { using type = ::message::conversion::math::imat9; };
    template <> struct ProtobufToMatrixMap<::umat9>  { using type = ::message::conversion::math::umat9; };
    template <> struct ProtobufToMatrixMap<::mat10>  { using type = ::message::conversion::math::mat10; };
    template <> struct ProtobufToMatrixMap<::fmat10> { using type = ::message::conversion::math::fmat10; };
    template <> struct ProtobufToMatrixMap<::imat10> { using type = ::message::conversion::math::imat10; };
    template <> struct ProtobufToMatrixMap<::umat10> { using type = ::message::conversion::math::umat10; };
    template <> struct ProtobufToMatrixMap<::mat11>  { using type = ::message::conversion::math::mat11; };
    template <> struct ProtobufToMatrixMap<::fmat11> { using type = ::message::conversion::math::fmat11; };
    template <> struct ProtobufToMatrixMap<::imat11> { using type = ::message::conversion::math::imat11; };
    template <> struct ProtobufToMatrixMap<::umat11> { using type = ::message::conversion::math::umat11; };
    template <> struct ProtobufToMatrixMap<::mat12>  { using type = ::message::conversion::math::mat12; };
    template <> struct ProtobufToMatrixMap<::fmat12> { using type = ::message::conversion::math::fmat12; };
    template <> struct ProtobufToMatrixMap<::imat12> { using type = ::message::conversion::math::imat12; };
    template <> struct ProtobufToMatrixMap<::umat12> { using type = ::message::conversion::math::umat12; };
    template <> struct ProtobufToMatrixMap<::mat13>  { using type = ::message::conversion::math::mat13; };
    template <> struct ProtobufToMatrixMap<::fmat13> { using type = ::message::conversion::math::fmat13; };
    template <> struct ProtobufToMatrixMap<::imat13> { using type = ::message::conversion::math::imat13; };
    template <> struct ProtobufToMatrixMap<::umat13> { using type = ::message::conversion::math::umat13; };
    template <> struct ProtobufToMatrixMap<::mat14>  { using type = ::message::conversion::math::mat14; };
    template <> struct ProtobufToMatrixMap<::fmat14> { using type = ::message::conversion::math::fmat14; };
    template <> struct ProtobufToMatrixMap<::imat14> { using type = ::message::conversion::math::imat14; };
    template <> struct ProtobufToMatrixMap<::umat14> { using type = ::message::conversion::math::umat14; };
    template <> struct ProtobufToMatrixMap<::mat15>  { using type = ::message::conversion::math::mat15; };
    template <> struct ProtobufToMatrixMap<::fmat15> { using type = ::message::conversion::math::fmat15; };
    template <> struct ProtobufToMatrixMap<::imat15> { using type = ::message::conversion::math::imat15; };
    template <> struct ProtobufToMatrixMap<::umat15> { using type = ::message::conversion::math::umat15; };
    template <> struct ProtobufToMatrixMap<::mat16>  { using type = ::message::conversion::math::mat16; };
    template <> struct ProtobufToMatrixMap<::fmat16> { using type = ::message::conversion::math::fmat16; };
    template <> struct ProtobufToMatrixMap<::imat16> { using type = ::message::conversion::math::imat16; };
    template <> struct ProtobufToMatrixMap<::umat16> { using type = ::message::conversion::math::umat16; };
    // clang-format on
    template <typename T>
    using MatProto = typename ProtobufToMatrixMap<T>::type;

    template <typename T>
    struct DynamicProtobufToMatrixMap;
    // clang-format off
    template <> struct DynamicProtobufToMatrixMap<::mat> { using type = ::message::conversion::math::mat; };
    template <> struct DynamicProtobufToMatrixMap<::fmat> { using type = ::message::conversion::math::fmat; };
    template <> struct DynamicProtobufToMatrixMap<::imat> { using type = ::message::conversion::math::imat; };
    template <> struct DynamicProtobufToMatrixMap<::umat> { using type = ::message::conversion::math::umat; };
    // clang-format on
    template <typename T>
    using DynamicMatProto = typename DynamicProtobufToMatrixMap<T>::type;

    /**
     * @brief This type maps matrix types to their protocol buffer equivalents
     *
     * @tparam T the matrix type to map
     */
    template <typename T>
    struct MatrixToProtobufMap;
    // clang-format off
    template <> struct MatrixToProtobufMap<::message::conversion::math::mat2>   { using type = ::mat2; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::fmat2>  { using type = ::fmat2; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::imat2>  { using type = ::imat2; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::umat2>  { using type = ::umat2; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::mat3>   { using type = ::mat3; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::fmat3>  { using type = ::fmat3; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::imat3>  { using type = ::imat3; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::umat3>  { using type = ::umat3; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::mat4>   { using type = ::mat4; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::fmat4>  { using type = ::fmat4; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::imat4>  { using type = ::imat4; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::umat4>  { using type = ::umat4; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::mat5>   { using type = ::mat5; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::fmat5>  { using type = ::fmat5; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::imat5>  { using type = ::imat5; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::umat5>  { using type = ::umat5; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::mat6>   { using type = ::mat6; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::fmat6>  { using type = ::fmat6; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::imat6>  { using type = ::imat6; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::umat6>  { using type = ::umat6; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::mat7>   { using type = ::mat7; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::fmat7>  { using type = ::fmat7; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::imat7>  { using type = ::imat7; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::umat7>  { using type = ::umat7; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::mat8>   { using type = ::mat8; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::fmat8>  { using type = ::fmat8; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::imat8>  { using type = ::imat8; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::umat8>  { using type = ::umat8; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::mat9>   { using type = ::mat9; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::fmat9>  { using type = ::fmat9; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::imat9>  { using type = ::imat9; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::umat9>  { using type = ::umat9; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::mat10>  { using type = ::mat10; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::fmat10> { using type = ::fmat10; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::imat10> { using type = ::imat10; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::umat10> { using type = ::umat10; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::mat11>  { using type = ::mat11; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::fmat11> { using type = ::fmat11; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::imat11> { using type = ::imat11; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::umat11> { using type = ::umat11; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::mat12>  { using type = ::mat12; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::fmat12> { using type = ::fmat12; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::imat12> { using type = ::imat12; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::umat12> { using type = ::umat12; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::mat13>  { using type = ::mat13; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::fmat13> { using type = ::fmat13; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::imat13> { using type = ::imat13; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::umat13> { using type = ::umat13; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::mat14>  { using type = ::mat14; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::fmat14> { using type = ::fmat14; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::imat14> { using type = ::imat14; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::umat14> { using type = ::umat14; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::mat15>  { using type = ::mat15; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::fmat15> { using type = ::fmat15; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::imat15> { using type = ::imat15; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::umat15> { using type = ::umat15; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::mat16>  { using type = ::mat16; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::fmat16> { using type = ::fmat16; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::imat16> { using type = ::imat16; };
    template <> struct MatrixToProtobufMap<::message::conversion::math::umat16> { using type = ::umat16; };
    // clang-format on
    template <typename T>
    using ProtoMat = typename MatrixToProtobufMap<T>::type;

    template <typename T>
    struct DynamicMatrixToProtobufMap;
    // clang-format off
    template <> struct DynamicMatrixToProtobufMap<::message::conversion::math::mat> { using type = ::mat; };
    template <> struct DynamicMatrixToProtobufMap<::message::conversion::math::fmat> { using type = ::fmat; };
    template <> struct DynamicMatrixToProtobufMap<::message::conversion::math::imat> { using type = ::imat; };
    template <> struct DynamicMatrixToProtobufMap<::message::conversion::math::umat> { using type = ::umat; };
    // clang-format on
    template <typename T>
    using DynamicProtoMat = typename DynamicMatrixToProtobufMap<T>::type;

    /**
     * @brief SFINAE functions to set protocol buffer values from vectors
     */
    namespace set_protobuf_from_vector {
        // clang-format off
        template <typename Proto, typename Vector> inline auto x(Proto& proto, const Vector& vector) -> decltype(proto.x(), void()) { proto.set_x(vector[0]); }
        template <typename... Args> inline void x(const Args&...) {}
        template <typename Proto, typename Vector> inline auto y(Proto& proto, const Vector& vector) -> decltype(proto.y(), void()) { proto.set_y(vector[1]); }
        template <typename... Args> inline void y(const Args&...) {}
        template <typename Proto, typename Vector> inline auto z(Proto& proto, const Vector& vector) -> decltype(proto.z(), void()) { proto.set_z(vector[2]); }
        template <typename... Args> inline void z(const Args&...) {}
        template <typename Proto, typename Vector> inline auto t(Proto& proto, const Vector& vector) -> decltype(proto.t(), void()) { proto.set_t(vector[3]); }
        template <typename... Args> inline void t(const Args&...) {}

        template <typename Proto, typename Vector> inline auto s0(Proto& proto, const Vector& vector) -> decltype(proto.s0(), void()) { proto.set_s0(vector[0]); }
        template <typename... Args> inline void s0(const Args&...) {}
        template <typename Proto, typename Vector> inline auto s1(Proto& proto, const Vector& vector) -> decltype(proto.s1(), void()) { proto.set_s1(vector[1]); }
        template <typename... Args> inline void s1(const Args&...) {}
        template <typename Proto, typename Vector> inline auto s2(Proto& proto, const Vector& vector) -> decltype(proto.s2(), void()) { proto.set_s2(vector[2]); }
        template <typename... Args> inline void s2(const Args&...) {}
        template <typename Proto, typename Vector> inline auto s3(Proto& proto, const Vector& vector) -> decltype(proto.s3(), void()) { proto.set_s3(vector[3]); }
        template <typename... Args> inline void s3(const Args&...) {}
        template <typename Proto, typename Vector> inline auto s4(Proto& proto, const Vector& vector) -> decltype(proto.s4(), void()) { proto.set_s4(vector[4]); }
        template <typename... Args> inline void s4(const Args&...) {}
        template <typename Proto, typename Vector> inline auto s5(Proto& proto, const Vector& vector) -> decltype(proto.s5(), void()) { proto.set_s5(vector[5]); }
        template <typename... Args> inline void s5(const Args&...) {}
        template <typename Proto, typename Vector> inline auto s6(Proto& proto, const Vector& vector) -> decltype(proto.s6(), void()) { proto.set_s6(vector[6]); }
        template <typename... Args> inline void s6(const Args&...) {}
        template <typename Proto, typename Vector> inline auto s7(Proto& proto, const Vector& vector) -> decltype(proto.s7(), void()) { proto.set_s7(vector[7]); }
        template <typename... Args> inline void s7(const Args&...) {}
        template <typename Proto, typename Vector> inline auto s8(Proto& proto, const Vector& vector) -> decltype(proto.s8(), void()) { proto.set_s8(vector[8]); }
        template <typename... Args> inline void s8(const Args&...) {}
        template <typename Proto, typename Vector> inline auto s9(Proto& proto, const Vector& vector) -> decltype(proto.s9(), void()) { proto.set_s9(vector[9]); }
        template <typename... Args> inline void s9(const Args&...) {}
        template <typename Proto, typename Vector> inline auto sa(Proto& proto, const Vector& vector) -> decltype(proto.sa(), void()) { proto.set_sa(vector[10]); }
        template <typename... Args> inline void sa(const Args&...) {}
        template <typename Proto, typename Vector> inline auto sb(Proto& proto, const Vector& vector) -> decltype(proto.sb(), void()) { proto.set_sb(vector[11]); }
        template <typename... Args> inline void sb(const Args&...) {}
        template <typename Proto, typename Vector> inline auto sc(Proto& proto, const Vector& vector) -> decltype(proto.sc(), void()) { proto.set_sc(vector[12]); }
        template <typename... Args> inline void sc(const Args&...) {}
        template <typename Proto, typename Vector> inline auto sd(Proto& proto, const Vector& vector) -> decltype(proto.sd(), void()) { proto.set_sd(vector[13]); }
        template <typename... Args> inline void sd(const Args&...) {}
        template <typename Proto, typename Vector> inline auto se(Proto& proto, const Vector& vector) -> decltype(proto.se(), void()) { proto.set_se(vector[14]); }
        template <typename... Args> inline void se(const Args&...) {}
        template <typename Proto, typename Vector> inline auto sf(Proto& proto, const Vector& vector) -> decltype(proto.sf(), void()) { proto.set_sf(vector[15]); }
        template <typename... Args> inline void sf(const Args&...) {}
        // clang-format on
    }  // namespace set_protobuf_from_vector

    /**
     * @brief SFINAE functions to set vector values from protocol buffers
     */
    namespace set_vector_from_protobuf {
        // clang-format off
        template <typename Proto, typename Vector> inline auto x(Vector&& vector, const Proto& proto) -> decltype(proto.x(), void()) { vector[0] = proto.x(); }
        template <typename... Args> inline void x(const Args&...) {}
        template <typename Proto, typename Vector> inline auto y(Vector&& vector, const Proto& proto) -> decltype(proto.y(), void()) { vector[1] = proto.y(); }
        template <typename... Args> inline void y(const Args&...) {}
        template <typename Proto, typename Vector> inline auto z(Vector&& vector, const Proto& proto) -> decltype(proto.z(), void()) { vector[2] = proto.z(); }
        template <typename... Args> inline void z(const Args&...) {}
        template <typename Proto, typename Vector> inline auto t(Vector&& vector, const Proto& proto) -> decltype(proto.t(), void()) { vector[3] = proto.t(); }
        template <typename... Args> inline void t(const Args&...) {}

        template <typename Proto, typename Vector> inline auto s0(Vector&& vector, const Proto& proto) -> decltype(proto.s0(), void()) { vector[0] = proto.s0(); }
        template <typename... Args> inline void s0(const Args&...) {}
        template <typename Proto, typename Vector> inline auto s1(Vector&& vector, const Proto& proto) -> decltype(proto.s1(), void()) { vector[1] = proto.s1(); }
        template <typename... Args> inline void s1(const Args&...) {}
        template <typename Proto, typename Vector> inline auto s2(Vector&& vector, const Proto& proto) -> decltype(proto.s2(), void()) { vector[2] = proto.s2(); }
        template <typename... Args> inline void s2(const Args&...) {}
        template <typename Proto, typename Vector> inline auto s3(Vector&& vector, const Proto& proto) -> decltype(proto.s3(), void()) { vector[3] = proto.s3(); }
        template <typename... Args> inline void s3(const Args&...) {}
        template <typename Proto, typename Vector> inline auto s4(Vector&& vector, const Proto& proto) -> decltype(proto.s4(), void()) { vector[4] = proto.s4(); }
        template <typename... Args> inline void s4(const Args&...) {}
        template <typename Proto, typename Vector> inline auto s5(Vector&& vector, const Proto& proto) -> decltype(proto.s5(), void()) { vector[5] = proto.s5(); }
        template <typename... Args> inline void s5(const Args&...) {}
        template <typename Proto, typename Vector> inline auto s6(Vector&& vector, const Proto& proto) -> decltype(proto.s6(), void()) { vector[6] = proto.s6(); }
        template <typename... Args> inline void s6(const Args&...) {}
        template <typename Proto, typename Vector> inline auto s7(Vector&& vector, const Proto& proto) -> decltype(proto.s7(), void()) { vector[7] = proto.s7(); }
        template <typename... Args> inline void s7(const Args&...) {}
        template <typename Proto, typename Vector> inline auto s8(Vector&& vector, const Proto& proto) -> decltype(proto.s8(), void()) { vector[8] = proto.s8(); }
        template <typename... Args> inline void s8(const Args&...) {}
        template <typename Proto, typename Vector> inline auto s9(Vector&& vector, const Proto& proto) -> decltype(proto.s9(), void()) { vector[9] = proto.s9(); }
        template <typename... Args> inline void s9(const Args&...) {}
        template <typename Proto, typename Vector> inline auto sa(Vector&& vector, const Proto& proto) -> decltype(proto.sa(), void()) { vector[10] = proto.sa(); }
        template <typename... Args> inline void sa(const Args&...) {}
        template <typename Proto, typename Vector> inline auto sb(Vector&& vector, const Proto& proto) -> decltype(proto.sb(), void()) { vector[11] = proto.sb(); }
        template <typename... Args> inline void sb(const Args&...) {}
        template <typename Proto, typename Vector> inline auto sc(Vector&& vector, const Proto& proto) -> decltype(proto.sc(), void()) { vector[12] = proto.sc(); }
        template <typename... Args> inline void sc(const Args&...) {}
        template <typename Proto, typename Vector> inline auto sd(Vector&& vector, const Proto& proto) -> decltype(proto.sd(), void()) { vector[13] = proto.sd(); }
        template <typename... Args> inline void sd(const Args&...) {}
        template <typename Proto, typename Vector> inline auto se(Vector&& vector, const Proto& proto) -> decltype(proto.se(), void()) { vector[14] = proto.se(); }
        template <typename... Args> inline void se(const Args&...) {}
        template <typename Proto, typename Vector> inline auto sf(Vector&& vector, const Proto& proto) -> decltype(proto.sf(), void()) { vector[15] = proto.sf(); }
        template <typename... Args> inline void sf(const Args&...) {}
        // clang-format on
    }  // namespace set_vector_from_protobuf

    /**
     * @brief SFINAE functions to set protocol buffer values
     */
    namespace set_protobuf_from_matrix {
        // mat2 - mat4
        template <typename Proto, typename Matrix>
        inline auto x(Proto& proto, const Matrix& matrix) -> decltype(proto.x(), void()) {
            set_protobuf_from_vector::x(*proto.mutable_x(), matrix.col(0));
            set_protobuf_from_vector::y(*proto.mutable_x(), matrix.col(0));
            set_protobuf_from_vector::z(*proto.mutable_x(), matrix.col(0));
            set_protobuf_from_vector::t(*proto.mutable_x(), matrix.col(0));
        }
        template <typename... Args>
        inline void x(const Args&...) {}

        template <typename Proto, typename Matrix>
        inline auto y(Proto& proto, const Matrix& matrix) -> decltype(proto.y(), void()) {
            set_protobuf_from_vector::x(*proto.mutable_y(), matrix.col(1));
            set_protobuf_from_vector::y(*proto.mutable_y(), matrix.col(1));
            set_protobuf_from_vector::z(*proto.mutable_y(), matrix.col(1));
            set_protobuf_from_vector::t(*proto.mutable_y(), matrix.col(1));
        }
        template <typename... Args>
        inline void y(const Args&...) {}

        template <typename Proto, typename Matrix>
        inline auto z(Proto& proto, const Matrix& matrix) -> decltype(proto.z(), void()) {
            set_protobuf_from_vector::x(*proto.mutable_z(), matrix.col(2));
            set_protobuf_from_vector::y(*proto.mutable_z(), matrix.col(2));
            set_protobuf_from_vector::z(*proto.mutable_z(), matrix.col(2));
            set_protobuf_from_vector::t(*proto.mutable_z(), matrix.col(2));
        }
        template <typename... Args>
        inline void z(const Args&...) {}

        template <typename Proto, typename Matrix>
        inline auto t(Proto& proto, const Matrix& matrix) -> decltype(proto.t(), void()) {
            set_protobuf_from_vector::x(*proto.mutable_t(), matrix.col(3));
            set_protobuf_from_vector::y(*proto.mutable_t(), matrix.col(3));
            set_protobuf_from_vector::z(*proto.mutable_t(), matrix.col(3));
            set_protobuf_from_vector::t(*proto.mutable_t(), matrix.col(3));
        }
        template <typename... Args>
        inline void t(const Args&...) {}

        // mat5 - mat16
        template <typename Proto, typename Matrix>
        inline auto s0(Proto& proto, const Matrix& matrix) -> decltype(proto.s0(), void()) {
            set_protobuf_from_vector::s0(*proto.mutable_s0(), matrix.col(0));
            set_protobuf_from_vector::s1(*proto.mutable_s0(), matrix.col(0));
            set_protobuf_from_vector::s2(*proto.mutable_s0(), matrix.col(0));
            set_protobuf_from_vector::s3(*proto.mutable_s0(), matrix.col(0));
            set_protobuf_from_vector::s4(*proto.mutable_s0(), matrix.col(0));
            set_protobuf_from_vector::s5(*proto.mutable_s0(), matrix.col(0));
            set_protobuf_from_vector::s6(*proto.mutable_s0(), matrix.col(0));
            set_protobuf_from_vector::s7(*proto.mutable_s0(), matrix.col(0));
            set_protobuf_from_vector::s8(*proto.mutable_s0(), matrix.col(0));
            set_protobuf_from_vector::s9(*proto.mutable_s0(), matrix.col(0));
            set_protobuf_from_vector::sa(*proto.mutable_s0(), matrix.col(0));
            set_protobuf_from_vector::sb(*proto.mutable_s0(), matrix.col(0));
            set_protobuf_from_vector::sc(*proto.mutable_s0(), matrix.col(0));
            set_protobuf_from_vector::sd(*proto.mutable_s0(), matrix.col(0));
            set_protobuf_from_vector::se(*proto.mutable_s0(), matrix.col(0));
            set_protobuf_from_vector::sf(*proto.mutable_s0(), matrix.col(0));
        }
        template <typename... Args>
        inline void s0(const Args&...) {}

        template <typename Proto, typename Matrix>
        inline auto s1(Proto& proto, const Matrix& matrix) -> decltype(proto.s1(), void()) {
            set_protobuf_from_vector::s0(*proto.mutable_s1(), matrix.col(1));
            set_protobuf_from_vector::s1(*proto.mutable_s1(), matrix.col(1));
            set_protobuf_from_vector::s2(*proto.mutable_s1(), matrix.col(1));
            set_protobuf_from_vector::s3(*proto.mutable_s1(), matrix.col(1));
            set_protobuf_from_vector::s4(*proto.mutable_s1(), matrix.col(1));
            set_protobuf_from_vector::s5(*proto.mutable_s1(), matrix.col(1));
            set_protobuf_from_vector::s6(*proto.mutable_s1(), matrix.col(1));
            set_protobuf_from_vector::s7(*proto.mutable_s1(), matrix.col(1));
            set_protobuf_from_vector::s8(*proto.mutable_s1(), matrix.col(1));
            set_protobuf_from_vector::s9(*proto.mutable_s1(), matrix.col(1));
            set_protobuf_from_vector::sa(*proto.mutable_s1(), matrix.col(1));
            set_protobuf_from_vector::sb(*proto.mutable_s1(), matrix.col(1));
            set_protobuf_from_vector::sc(*proto.mutable_s1(), matrix.col(1));
            set_protobuf_from_vector::sd(*proto.mutable_s1(), matrix.col(1));
            set_protobuf_from_vector::se(*proto.mutable_s1(), matrix.col(1));
            set_protobuf_from_vector::sf(*proto.mutable_s1(), matrix.col(1));
        }
        template <typename... Args>
        inline void s1(const Args&...) {}

        template <typename Proto, typename Matrix>
        inline auto s2(Proto& proto, const Matrix& matrix) -> decltype(proto.s2(), void()) {
            set_protobuf_from_vector::s0(*proto.mutable_s2(), matrix.col(2));
            set_protobuf_from_vector::s1(*proto.mutable_s2(), matrix.col(2));
            set_protobuf_from_vector::s2(*proto.mutable_s2(), matrix.col(2));
            set_protobuf_from_vector::s3(*proto.mutable_s2(), matrix.col(2));
            set_protobuf_from_vector::s4(*proto.mutable_s2(), matrix.col(2));
            set_protobuf_from_vector::s5(*proto.mutable_s2(), matrix.col(2));
            set_protobuf_from_vector::s6(*proto.mutable_s2(), matrix.col(2));
            set_protobuf_from_vector::s7(*proto.mutable_s2(), matrix.col(2));
            set_protobuf_from_vector::s8(*proto.mutable_s2(), matrix.col(2));
            set_protobuf_from_vector::s9(*proto.mutable_s2(), matrix.col(2));
            set_protobuf_from_vector::sa(*proto.mutable_s2(), matrix.col(2));
            set_protobuf_from_vector::sb(*proto.mutable_s2(), matrix.col(2));
            set_protobuf_from_vector::sc(*proto.mutable_s2(), matrix.col(2));
            set_protobuf_from_vector::sd(*proto.mutable_s2(), matrix.col(2));
            set_protobuf_from_vector::se(*proto.mutable_s2(), matrix.col(2));
            set_protobuf_from_vector::sf(*proto.mutable_s2(), matrix.col(2));
        }
        template <typename... Args>
        inline void s2(const Args&...) {}

        template <typename Proto, typename Matrix>
        inline auto s3(Proto& proto, const Matrix& matrix) -> decltype(proto.s3(), void()) {
            set_protobuf_from_vector::s0(*proto.mutable_s3(), matrix.col(3));
            set_protobuf_from_vector::s1(*proto.mutable_s3(), matrix.col(3));
            set_protobuf_from_vector::s2(*proto.mutable_s3(), matrix.col(3));
            set_protobuf_from_vector::s3(*proto.mutable_s3(), matrix.col(3));
            set_protobuf_from_vector::s4(*proto.mutable_s3(), matrix.col(3));
            set_protobuf_from_vector::s5(*proto.mutable_s3(), matrix.col(3));
            set_protobuf_from_vector::s6(*proto.mutable_s3(), matrix.col(3));
            set_protobuf_from_vector::s7(*proto.mutable_s3(), matrix.col(3));
            set_protobuf_from_vector::s8(*proto.mutable_s3(), matrix.col(3));
            set_protobuf_from_vector::s9(*proto.mutable_s3(), matrix.col(3));
            set_protobuf_from_vector::sa(*proto.mutable_s3(), matrix.col(3));
            set_protobuf_from_vector::sb(*proto.mutable_s3(), matrix.col(3));
            set_protobuf_from_vector::sc(*proto.mutable_s3(), matrix.col(3));
            set_protobuf_from_vector::sd(*proto.mutable_s3(), matrix.col(3));
            set_protobuf_from_vector::se(*proto.mutable_s3(), matrix.col(3));
            set_protobuf_from_vector::sf(*proto.mutable_s3(), matrix.col(3));
        }
        template <typename... Args>
        inline void s3(const Args&...) {}

        template <typename Proto, typename Matrix>
        inline auto s4(Proto& proto, const Matrix& matrix) -> decltype(proto.s4(), void()) {
            set_protobuf_from_vector::s0(*proto.mutable_s4(), matrix.col(4));
            set_protobuf_from_vector::s1(*proto.mutable_s4(), matrix.col(4));
            set_protobuf_from_vector::s2(*proto.mutable_s4(), matrix.col(4));
            set_protobuf_from_vector::s3(*proto.mutable_s4(), matrix.col(4));
            set_protobuf_from_vector::s4(*proto.mutable_s4(), matrix.col(4));
            set_protobuf_from_vector::s5(*proto.mutable_s4(), matrix.col(4));
            set_protobuf_from_vector::s6(*proto.mutable_s4(), matrix.col(4));
            set_protobuf_from_vector::s7(*proto.mutable_s4(), matrix.col(4));
            set_protobuf_from_vector::s8(*proto.mutable_s4(), matrix.col(4));
            set_protobuf_from_vector::s9(*proto.mutable_s4(), matrix.col(4));
            set_protobuf_from_vector::sa(*proto.mutable_s4(), matrix.col(4));
            set_protobuf_from_vector::sb(*proto.mutable_s4(), matrix.col(4));
            set_protobuf_from_vector::sc(*proto.mutable_s4(), matrix.col(4));
            set_protobuf_from_vector::sd(*proto.mutable_s4(), matrix.col(4));
            set_protobuf_from_vector::se(*proto.mutable_s4(), matrix.col(4));
            set_protobuf_from_vector::sf(*proto.mutable_s4(), matrix.col(4));
        }
        template <typename... Args>
        inline void s4(const Args&...) {}

        template <typename Proto, typename Matrix>
        inline auto s5(Proto& proto, const Matrix& matrix) -> decltype(proto.s5(), void()) {
            set_protobuf_from_vector::s0(*proto.mutable_s5(), matrix.col(5));
            set_protobuf_from_vector::s1(*proto.mutable_s5(), matrix.col(5));
            set_protobuf_from_vector::s2(*proto.mutable_s5(), matrix.col(5));
            set_protobuf_from_vector::s3(*proto.mutable_s5(), matrix.col(5));
            set_protobuf_from_vector::s4(*proto.mutable_s5(), matrix.col(5));
            set_protobuf_from_vector::s5(*proto.mutable_s5(), matrix.col(5));
            set_protobuf_from_vector::s6(*proto.mutable_s5(), matrix.col(5));
            set_protobuf_from_vector::s7(*proto.mutable_s5(), matrix.col(5));
            set_protobuf_from_vector::s8(*proto.mutable_s5(), matrix.col(5));
            set_protobuf_from_vector::s9(*proto.mutable_s5(), matrix.col(5));
            set_protobuf_from_vector::sa(*proto.mutable_s5(), matrix.col(5));
            set_protobuf_from_vector::sb(*proto.mutable_s5(), matrix.col(5));
            set_protobuf_from_vector::sc(*proto.mutable_s5(), matrix.col(5));
            set_protobuf_from_vector::sd(*proto.mutable_s5(), matrix.col(5));
            set_protobuf_from_vector::se(*proto.mutable_s5(), matrix.col(5));
            set_protobuf_from_vector::sf(*proto.mutable_s5(), matrix.col(5));
        }
        template <typename... Args>
        inline void s5(const Args&...) {}

        template <typename Proto, typename Matrix>
        inline auto s6(Proto& proto, const Matrix& matrix) -> decltype(proto.s6(), void()) {
            set_protobuf_from_vector::s0(*proto.mutable_s6(), matrix.col(6));
            set_protobuf_from_vector::s1(*proto.mutable_s6(), matrix.col(6));
            set_protobuf_from_vector::s2(*proto.mutable_s6(), matrix.col(6));
            set_protobuf_from_vector::s3(*proto.mutable_s6(), matrix.col(6));
            set_protobuf_from_vector::s4(*proto.mutable_s6(), matrix.col(6));
            set_protobuf_from_vector::s5(*proto.mutable_s6(), matrix.col(6));
            set_protobuf_from_vector::s6(*proto.mutable_s6(), matrix.col(6));
            set_protobuf_from_vector::s7(*proto.mutable_s6(), matrix.col(6));
            set_protobuf_from_vector::s8(*proto.mutable_s6(), matrix.col(6));
            set_protobuf_from_vector::s9(*proto.mutable_s6(), matrix.col(6));
            set_protobuf_from_vector::sa(*proto.mutable_s6(), matrix.col(6));
            set_protobuf_from_vector::sb(*proto.mutable_s6(), matrix.col(6));
            set_protobuf_from_vector::sc(*proto.mutable_s6(), matrix.col(6));
            set_protobuf_from_vector::sd(*proto.mutable_s6(), matrix.col(6));
            set_protobuf_from_vector::se(*proto.mutable_s6(), matrix.col(6));
            set_protobuf_from_vector::sf(*proto.mutable_s6(), matrix.col(6));
        }
        template <typename... Args>
        inline void s6(const Args&...) {}

        template <typename Proto, typename Matrix>
        inline auto s7(Proto& proto, const Matrix& matrix) -> decltype(proto.s7(), void()) {
            set_protobuf_from_vector::s0(*proto.mutable_s7(), matrix.col(7));
            set_protobuf_from_vector::s1(*proto.mutable_s7(), matrix.col(7));
            set_protobuf_from_vector::s2(*proto.mutable_s7(), matrix.col(7));
            set_protobuf_from_vector::s3(*proto.mutable_s7(), matrix.col(7));
            set_protobuf_from_vector::s4(*proto.mutable_s7(), matrix.col(7));
            set_protobuf_from_vector::s5(*proto.mutable_s7(), matrix.col(7));
            set_protobuf_from_vector::s6(*proto.mutable_s7(), matrix.col(7));
            set_protobuf_from_vector::s7(*proto.mutable_s7(), matrix.col(7));
            set_protobuf_from_vector::s8(*proto.mutable_s7(), matrix.col(7));
            set_protobuf_from_vector::s9(*proto.mutable_s7(), matrix.col(7));
            set_protobuf_from_vector::sa(*proto.mutable_s7(), matrix.col(7));
            set_protobuf_from_vector::sb(*proto.mutable_s7(), matrix.col(7));
            set_protobuf_from_vector::sc(*proto.mutable_s7(), matrix.col(7));
            set_protobuf_from_vector::sd(*proto.mutable_s7(), matrix.col(7));
            set_protobuf_from_vector::se(*proto.mutable_s7(), matrix.col(7));
            set_protobuf_from_vector::sf(*proto.mutable_s7(), matrix.col(7));
        }
        template <typename... Args>
        inline void s7(const Args&...) {}

        template <typename Proto, typename Matrix>
        inline auto s8(Proto& proto, const Matrix& matrix) -> decltype(proto.s8(), void()) {
            set_protobuf_from_vector::s0(*proto.mutable_s8(), matrix.col(8));
            set_protobuf_from_vector::s1(*proto.mutable_s8(), matrix.col(8));
            set_protobuf_from_vector::s2(*proto.mutable_s8(), matrix.col(8));
            set_protobuf_from_vector::s3(*proto.mutable_s8(), matrix.col(8));
            set_protobuf_from_vector::s4(*proto.mutable_s8(), matrix.col(8));
            set_protobuf_from_vector::s5(*proto.mutable_s8(), matrix.col(8));
            set_protobuf_from_vector::s6(*proto.mutable_s8(), matrix.col(8));
            set_protobuf_from_vector::s7(*proto.mutable_s8(), matrix.col(8));
            set_protobuf_from_vector::s8(*proto.mutable_s8(), matrix.col(8));
            set_protobuf_from_vector::s9(*proto.mutable_s8(), matrix.col(8));
            set_protobuf_from_vector::sa(*proto.mutable_s8(), matrix.col(8));
            set_protobuf_from_vector::sb(*proto.mutable_s8(), matrix.col(8));
            set_protobuf_from_vector::sc(*proto.mutable_s8(), matrix.col(8));
            set_protobuf_from_vector::sd(*proto.mutable_s8(), matrix.col(8));
            set_protobuf_from_vector::se(*proto.mutable_s8(), matrix.col(8));
            set_protobuf_from_vector::sf(*proto.mutable_s8(), matrix.col(8));
        }
        template <typename... Args>
        inline void s8(const Args&...) {}

        template <typename Proto, typename Matrix>
        inline auto s9(Proto& proto, const Matrix& matrix) -> decltype(proto.s9(), void()) {
            set_protobuf_from_vector::s0(*proto.mutable_s9(), matrix.col(9));
            set_protobuf_from_vector::s1(*proto.mutable_s9(), matrix.col(9));
            set_protobuf_from_vector::s2(*proto.mutable_s9(), matrix.col(9));
            set_protobuf_from_vector::s3(*proto.mutable_s9(), matrix.col(9));
            set_protobuf_from_vector::s4(*proto.mutable_s9(), matrix.col(9));
            set_protobuf_from_vector::s5(*proto.mutable_s9(), matrix.col(9));
            set_protobuf_from_vector::s6(*proto.mutable_s9(), matrix.col(9));
            set_protobuf_from_vector::s7(*proto.mutable_s9(), matrix.col(9));
            set_protobuf_from_vector::s8(*proto.mutable_s9(), matrix.col(9));
            set_protobuf_from_vector::s9(*proto.mutable_s9(), matrix.col(9));
            set_protobuf_from_vector::sa(*proto.mutable_s9(), matrix.col(9));
            set_protobuf_from_vector::sb(*proto.mutable_s9(), matrix.col(9));
            set_protobuf_from_vector::sc(*proto.mutable_s9(), matrix.col(9));
            set_protobuf_from_vector::sd(*proto.mutable_s9(), matrix.col(9));
            set_protobuf_from_vector::se(*proto.mutable_s9(), matrix.col(9));
            set_protobuf_from_vector::sf(*proto.mutable_s9(), matrix.col(9));
        }
        template <typename... Args>
        inline void s9(const Args&...) {}

        template <typename Proto, typename Matrix>
        inline auto sa(Proto& proto, const Matrix& matrix) -> decltype(proto.sa(), void()) {
            set_protobuf_from_vector::s0(*proto.mutable_sa(), matrix.col(10));
            set_protobuf_from_vector::s1(*proto.mutable_sa(), matrix.col(10));
            set_protobuf_from_vector::s2(*proto.mutable_sa(), matrix.col(10));
            set_protobuf_from_vector::s3(*proto.mutable_sa(), matrix.col(10));
            set_protobuf_from_vector::s4(*proto.mutable_sa(), matrix.col(10));
            set_protobuf_from_vector::s5(*proto.mutable_sa(), matrix.col(10));
            set_protobuf_from_vector::s6(*proto.mutable_sa(), matrix.col(10));
            set_protobuf_from_vector::s7(*proto.mutable_sa(), matrix.col(10));
            set_protobuf_from_vector::s8(*proto.mutable_sa(), matrix.col(10));
            set_protobuf_from_vector::s9(*proto.mutable_sa(), matrix.col(10));
            set_protobuf_from_vector::sa(*proto.mutable_sa(), matrix.col(10));
            set_protobuf_from_vector::sb(*proto.mutable_sa(), matrix.col(10));
            set_protobuf_from_vector::sc(*proto.mutable_sa(), matrix.col(10));
            set_protobuf_from_vector::sd(*proto.mutable_sa(), matrix.col(10));
            set_protobuf_from_vector::se(*proto.mutable_sa(), matrix.col(10));
            set_protobuf_from_vector::sf(*proto.mutable_sa(), matrix.col(10));
        }
        template <typename... Args>
        inline void sa(const Args&...) {}

        template <typename Proto, typename Matrix>
        inline auto sb(Proto& proto, const Matrix& matrix) -> decltype(proto.sb(), void()) {
            set_protobuf_from_vector::s0(*proto.mutable_sb(), matrix.col(11));
            set_protobuf_from_vector::s1(*proto.mutable_sb(), matrix.col(11));
            set_protobuf_from_vector::s2(*proto.mutable_sb(), matrix.col(11));
            set_protobuf_from_vector::s3(*proto.mutable_sb(), matrix.col(11));
            set_protobuf_from_vector::s4(*proto.mutable_sb(), matrix.col(11));
            set_protobuf_from_vector::s5(*proto.mutable_sb(), matrix.col(11));
            set_protobuf_from_vector::s6(*proto.mutable_sb(), matrix.col(11));
            set_protobuf_from_vector::s7(*proto.mutable_sb(), matrix.col(11));
            set_protobuf_from_vector::s8(*proto.mutable_sb(), matrix.col(11));
            set_protobuf_from_vector::s9(*proto.mutable_sb(), matrix.col(11));
            set_protobuf_from_vector::sa(*proto.mutable_sb(), matrix.col(11));
            set_protobuf_from_vector::sb(*proto.mutable_sb(), matrix.col(11));
            set_protobuf_from_vector::sc(*proto.mutable_sb(), matrix.col(11));
            set_protobuf_from_vector::sd(*proto.mutable_sb(), matrix.col(11));
            set_protobuf_from_vector::se(*proto.mutable_sb(), matrix.col(11));
            set_protobuf_from_vector::sf(*proto.mutable_sb(), matrix.col(11));
        }
        template <typename... Args>
        inline void sb(const Args&...) {}

        template <typename Proto, typename Matrix>
        inline auto sc(Proto& proto, const Matrix& matrix) -> decltype(proto.sc(), void()) {
            set_protobuf_from_vector::s0(*proto.mutable_sc(), matrix.col(12));
            set_protobuf_from_vector::s1(*proto.mutable_sc(), matrix.col(12));
            set_protobuf_from_vector::s2(*proto.mutable_sc(), matrix.col(12));
            set_protobuf_from_vector::s3(*proto.mutable_sc(), matrix.col(12));
            set_protobuf_from_vector::s4(*proto.mutable_sc(), matrix.col(12));
            set_protobuf_from_vector::s5(*proto.mutable_sc(), matrix.col(12));
            set_protobuf_from_vector::s6(*proto.mutable_sc(), matrix.col(12));
            set_protobuf_from_vector::s7(*proto.mutable_sc(), matrix.col(12));
            set_protobuf_from_vector::s8(*proto.mutable_sc(), matrix.col(12));
            set_protobuf_from_vector::s9(*proto.mutable_sc(), matrix.col(12));
            set_protobuf_from_vector::sa(*proto.mutable_sc(), matrix.col(12));
            set_protobuf_from_vector::sb(*proto.mutable_sc(), matrix.col(12));
            set_protobuf_from_vector::sc(*proto.mutable_sc(), matrix.col(12));
            set_protobuf_from_vector::sd(*proto.mutable_sc(), matrix.col(12));
            set_protobuf_from_vector::se(*proto.mutable_sc(), matrix.col(12));
            set_protobuf_from_vector::sf(*proto.mutable_sc(), matrix.col(12));
        }
        template <typename... Args>
        inline void sc(const Args&...) {}

        template <typename Proto, typename Matrix>
        inline auto sd(Proto& proto, const Matrix& matrix) -> decltype(proto.sd(), void()) {
            set_protobuf_from_vector::s0(*proto.mutable_sd(), matrix.col(13));
            set_protobuf_from_vector::s1(*proto.mutable_sd(), matrix.col(13));
            set_protobuf_from_vector::s2(*proto.mutable_sd(), matrix.col(13));
            set_protobuf_from_vector::s3(*proto.mutable_sd(), matrix.col(13));
            set_protobuf_from_vector::s4(*proto.mutable_sd(), matrix.col(13));
            set_protobuf_from_vector::s5(*proto.mutable_sd(), matrix.col(13));
            set_protobuf_from_vector::s6(*proto.mutable_sd(), matrix.col(13));
            set_protobuf_from_vector::s7(*proto.mutable_sd(), matrix.col(13));
            set_protobuf_from_vector::s8(*proto.mutable_sd(), matrix.col(13));
            set_protobuf_from_vector::s9(*proto.mutable_sd(), matrix.col(13));
            set_protobuf_from_vector::sa(*proto.mutable_sd(), matrix.col(13));
            set_protobuf_from_vector::sb(*proto.mutable_sd(), matrix.col(13));
            set_protobuf_from_vector::sc(*proto.mutable_sd(), matrix.col(13));
            set_protobuf_from_vector::sd(*proto.mutable_sd(), matrix.col(13));
            set_protobuf_from_vector::se(*proto.mutable_sd(), matrix.col(13));
            set_protobuf_from_vector::sf(*proto.mutable_sd(), matrix.col(13));
        }
        template <typename... Args>
        inline void sd(const Args&...) {}

        template <typename Proto, typename Matrix>
        inline auto se(Proto& proto, const Matrix& matrix) -> decltype(proto.se(), void()) {
            set_protobuf_from_vector::s0(*proto.mutable_se(), matrix.col(14));
            set_protobuf_from_vector::s1(*proto.mutable_se(), matrix.col(14));
            set_protobuf_from_vector::s2(*proto.mutable_se(), matrix.col(14));
            set_protobuf_from_vector::s3(*proto.mutable_se(), matrix.col(14));
            set_protobuf_from_vector::s4(*proto.mutable_se(), matrix.col(14));
            set_protobuf_from_vector::s5(*proto.mutable_se(), matrix.col(14));
            set_protobuf_from_vector::s6(*proto.mutable_se(), matrix.col(14));
            set_protobuf_from_vector::s7(*proto.mutable_se(), matrix.col(14));
            set_protobuf_from_vector::s8(*proto.mutable_se(), matrix.col(14));
            set_protobuf_from_vector::s9(*proto.mutable_se(), matrix.col(14));
            set_protobuf_from_vector::sa(*proto.mutable_se(), matrix.col(14));
            set_protobuf_from_vector::sb(*proto.mutable_se(), matrix.col(14));
            set_protobuf_from_vector::sc(*proto.mutable_se(), matrix.col(14));
            set_protobuf_from_vector::sd(*proto.mutable_se(), matrix.col(14));
            set_protobuf_from_vector::se(*proto.mutable_se(), matrix.col(14));
            set_protobuf_from_vector::sf(*proto.mutable_se(), matrix.col(14));
        }
        template <typename... Args>
        inline void se(const Args&...) {}

        template <typename Proto, typename Matrix>
        inline auto sf(Proto& proto, const Matrix& matrix) -> decltype(proto.sf(), void()) {
            set_protobuf_from_vector::s0(*proto.mutable_sf(), matrix.col(15));
            set_protobuf_from_vector::s1(*proto.mutable_sf(), matrix.col(15));
            set_protobuf_from_vector::s2(*proto.mutable_sf(), matrix.col(15));
            set_protobuf_from_vector::s3(*proto.mutable_sf(), matrix.col(15));
            set_protobuf_from_vector::s4(*proto.mutable_sf(), matrix.col(15));
            set_protobuf_from_vector::s5(*proto.mutable_sf(), matrix.col(15));
            set_protobuf_from_vector::s6(*proto.mutable_sf(), matrix.col(15));
            set_protobuf_from_vector::s7(*proto.mutable_sf(), matrix.col(15));
            set_protobuf_from_vector::s8(*proto.mutable_sf(), matrix.col(15));
            set_protobuf_from_vector::s9(*proto.mutable_sf(), matrix.col(15));
            set_protobuf_from_vector::sa(*proto.mutable_sf(), matrix.col(15));
            set_protobuf_from_vector::sb(*proto.mutable_sf(), matrix.col(15));
            set_protobuf_from_vector::sc(*proto.mutable_sf(), matrix.col(15));
            set_protobuf_from_vector::sd(*proto.mutable_sf(), matrix.col(15));
            set_protobuf_from_vector::se(*proto.mutable_sf(), matrix.col(15));
            set_protobuf_from_vector::sf(*proto.mutable_sf(), matrix.col(15));
        }
        template <typename... Args>
        inline void sf(const Args&...) {}

    }  // namespace set_protobuf_from_matrix

    /**
     * @brief SFINAE functions to set matrix values
     */
    namespace set_matrix_from_protobuf {
        // mat2 - mat4
        template <typename Proto, typename Matrix>
        inline auto x(Matrix& matrix, const Proto& proto) -> decltype(proto.x(), void()) {
            set_vector_from_protobuf::x(matrix.col(0), proto.x());
            set_vector_from_protobuf::y(matrix.col(0), proto.x());
            set_vector_from_protobuf::z(matrix.col(0), proto.x());
            set_vector_from_protobuf::t(matrix.col(0), proto.x());
        }
        template <typename... Args>
        inline void x(Args&...) {}

        template <typename Proto, typename Matrix>
        inline auto y(Matrix& matrix, const Proto& proto) -> decltype(proto.y(), void()) {
            set_vector_from_protobuf::x(matrix.col(1), proto.y());
            set_vector_from_protobuf::y(matrix.col(1), proto.y());
            set_vector_from_protobuf::z(matrix.col(1), proto.y());
            set_vector_from_protobuf::t(matrix.col(1), proto.y());
        }
        template <typename... Args>
        inline void y(Args&...) {}

        template <typename Proto, typename Matrix>
        inline auto z(Matrix& matrix, const Proto& proto) -> decltype(proto.z(), void()) {
            set_vector_from_protobuf::x(matrix.col(2), proto.z());
            set_vector_from_protobuf::y(matrix.col(2), proto.z());
            set_vector_from_protobuf::z(matrix.col(2), proto.z());
            set_vector_from_protobuf::t(matrix.col(2), proto.z());
        }
        template <typename... Args>
        inline void z(Args&...) {}

        template <typename Proto, typename Matrix>
        inline auto t(Matrix& matrix, const Proto& proto) -> decltype(proto.t(), void()) {
            set_vector_from_protobuf::x(matrix.col(3), proto.t());
            set_vector_from_protobuf::y(matrix.col(3), proto.t());
            set_vector_from_protobuf::z(matrix.col(3), proto.t());
            set_vector_from_protobuf::t(matrix.col(3), proto.t());
        }
        template <typename... Args>
        inline void t(Args&...) {}

        // mat5 - mat16
        template <typename Proto, typename Matrix>
        inline auto s0(Matrix& matrix, const Proto& proto) -> decltype(proto.s0(), void()) {
            set_vector_from_protobuf::s0(matrix.col(0), proto.s0());
            set_vector_from_protobuf::s1(matrix.col(0), proto.s0());
            set_vector_from_protobuf::s2(matrix.col(0), proto.s0());
            set_vector_from_protobuf::s3(matrix.col(0), proto.s0());
            set_vector_from_protobuf::s4(matrix.col(0), proto.s0());
            set_vector_from_protobuf::s5(matrix.col(0), proto.s0());
            set_vector_from_protobuf::s6(matrix.col(0), proto.s0());
            set_vector_from_protobuf::s7(matrix.col(0), proto.s0());
            set_vector_from_protobuf::s8(matrix.col(0), proto.s0());
            set_vector_from_protobuf::s9(matrix.col(0), proto.s0());
            set_vector_from_protobuf::sa(matrix.col(0), proto.s0());
            set_vector_from_protobuf::sb(matrix.col(0), proto.s0());
            set_vector_from_protobuf::sc(matrix.col(0), proto.s0());
            set_vector_from_protobuf::sd(matrix.col(0), proto.s0());
            set_vector_from_protobuf::se(matrix.col(0), proto.s0());
            set_vector_from_protobuf::sf(matrix.col(0), proto.s0());
        }
        template <typename... Args>
        inline void s0(const Args&...) {}

        template <typename Proto, typename Matrix>
        inline auto s1(Matrix& matrix, const Proto& proto) -> decltype(proto.s1(), void()) {
            set_vector_from_protobuf::s0(matrix.col(1), proto.s1());
            set_vector_from_protobuf::s1(matrix.col(1), proto.s1());
            set_vector_from_protobuf::s2(matrix.col(1), proto.s1());
            set_vector_from_protobuf::s3(matrix.col(1), proto.s1());
            set_vector_from_protobuf::s4(matrix.col(1), proto.s1());
            set_vector_from_protobuf::s5(matrix.col(1), proto.s1());
            set_vector_from_protobuf::s6(matrix.col(1), proto.s1());
            set_vector_from_protobuf::s7(matrix.col(1), proto.s1());
            set_vector_from_protobuf::s8(matrix.col(1), proto.s1());
            set_vector_from_protobuf::s9(matrix.col(1), proto.s1());
            set_vector_from_protobuf::sa(matrix.col(1), proto.s1());
            set_vector_from_protobuf::sb(matrix.col(1), proto.s1());
            set_vector_from_protobuf::sc(matrix.col(1), proto.s1());
            set_vector_from_protobuf::sd(matrix.col(1), proto.s1());
            set_vector_from_protobuf::se(matrix.col(1), proto.s1());
            set_vector_from_protobuf::sf(matrix.col(1), proto.s1());
        }
        template <typename... Args>
        inline void s1(const Args&...) {}

        template <typename Proto, typename Matrix>
        inline auto s2(Matrix& matrix, const Proto& proto) -> decltype(proto.s2(), void()) {
            set_vector_from_protobuf::s0(matrix.col(2), proto.s2());
            set_vector_from_protobuf::s1(matrix.col(2), proto.s2());
            set_vector_from_protobuf::s2(matrix.col(2), proto.s2());
            set_vector_from_protobuf::s3(matrix.col(2), proto.s2());
            set_vector_from_protobuf::s4(matrix.col(2), proto.s2());
            set_vector_from_protobuf::s5(matrix.col(2), proto.s2());
            set_vector_from_protobuf::s6(matrix.col(2), proto.s2());
            set_vector_from_protobuf::s7(matrix.col(2), proto.s2());
            set_vector_from_protobuf::s8(matrix.col(2), proto.s2());
            set_vector_from_protobuf::s9(matrix.col(2), proto.s2());
            set_vector_from_protobuf::sa(matrix.col(2), proto.s2());
            set_vector_from_protobuf::sb(matrix.col(2), proto.s2());
            set_vector_from_protobuf::sc(matrix.col(2), proto.s2());
            set_vector_from_protobuf::sd(matrix.col(2), proto.s2());
            set_vector_from_protobuf::se(matrix.col(2), proto.s2());
            set_vector_from_protobuf::sf(matrix.col(2), proto.s2());
        }
        template <typename... Args>
        inline void s2(const Args&...) {}

        template <typename Proto, typename Matrix>
        inline auto s3(Matrix& matrix, const Proto& proto) -> decltype(proto.s3(), void()) {
            set_vector_from_protobuf::s0(matrix.col(3), proto.s3());
            set_vector_from_protobuf::s1(matrix.col(3), proto.s3());
            set_vector_from_protobuf::s2(matrix.col(3), proto.s3());
            set_vector_from_protobuf::s3(matrix.col(3), proto.s3());
            set_vector_from_protobuf::s4(matrix.col(3), proto.s3());
            set_vector_from_protobuf::s5(matrix.col(3), proto.s3());
            set_vector_from_protobuf::s6(matrix.col(3), proto.s3());
            set_vector_from_protobuf::s7(matrix.col(3), proto.s3());
            set_vector_from_protobuf::s8(matrix.col(3), proto.s3());
            set_vector_from_protobuf::s9(matrix.col(3), proto.s3());
            set_vector_from_protobuf::sa(matrix.col(3), proto.s3());
            set_vector_from_protobuf::sb(matrix.col(3), proto.s3());
            set_vector_from_protobuf::sc(matrix.col(3), proto.s3());
            set_vector_from_protobuf::sd(matrix.col(3), proto.s3());
            set_vector_from_protobuf::se(matrix.col(3), proto.s3());
            set_vector_from_protobuf::sf(matrix.col(3), proto.s3());
        }
        template <typename... Args>
        inline void s3(const Args&...) {}

        template <typename Proto, typename Matrix>
        inline auto s4(Matrix& matrix, const Proto& proto) -> decltype(proto.s4(), void()) {
            set_vector_from_protobuf::s0(matrix.col(4), proto.s4());
            set_vector_from_protobuf::s1(matrix.col(4), proto.s4());
            set_vector_from_protobuf::s2(matrix.col(4), proto.s4());
            set_vector_from_protobuf::s3(matrix.col(4), proto.s4());
            set_vector_from_protobuf::s4(matrix.col(4), proto.s4());
            set_vector_from_protobuf::s5(matrix.col(4), proto.s4());
            set_vector_from_protobuf::s6(matrix.col(4), proto.s4());
            set_vector_from_protobuf::s7(matrix.col(4), proto.s4());
            set_vector_from_protobuf::s8(matrix.col(4), proto.s4());
            set_vector_from_protobuf::s9(matrix.col(4), proto.s4());
            set_vector_from_protobuf::sa(matrix.col(4), proto.s4());
            set_vector_from_protobuf::sb(matrix.col(4), proto.s4());
            set_vector_from_protobuf::sc(matrix.col(4), proto.s4());
            set_vector_from_protobuf::sd(matrix.col(4), proto.s4());
            set_vector_from_protobuf::se(matrix.col(4), proto.s4());
            set_vector_from_protobuf::sf(matrix.col(4), proto.s4());
        }
        template <typename... Args>
        inline void s4(const Args&...) {}

        template <typename Proto, typename Matrix>
        inline auto s5(Matrix& matrix, const Proto& proto) -> decltype(proto.s5(), void()) {
            set_vector_from_protobuf::s0(matrix.col(5), proto.s5());
            set_vector_from_protobuf::s1(matrix.col(5), proto.s5());
            set_vector_from_protobuf::s2(matrix.col(5), proto.s5());
            set_vector_from_protobuf::s3(matrix.col(5), proto.s5());
            set_vector_from_protobuf::s4(matrix.col(5), proto.s5());
            set_vector_from_protobuf::s5(matrix.col(5), proto.s5());
            set_vector_from_protobuf::s6(matrix.col(5), proto.s5());
            set_vector_from_protobuf::s7(matrix.col(5), proto.s5());
            set_vector_from_protobuf::s8(matrix.col(5), proto.s5());
            set_vector_from_protobuf::s9(matrix.col(5), proto.s5());
            set_vector_from_protobuf::sa(matrix.col(5), proto.s5());
            set_vector_from_protobuf::sb(matrix.col(5), proto.s5());
            set_vector_from_protobuf::sc(matrix.col(5), proto.s5());
            set_vector_from_protobuf::sd(matrix.col(5), proto.s5());
            set_vector_from_protobuf::se(matrix.col(5), proto.s5());
            set_vector_from_protobuf::sf(matrix.col(5), proto.s5());
        }
        template <typename... Args>
        inline void s5(const Args&...) {}

        template <typename Proto, typename Matrix>
        inline auto s6(Matrix& matrix, const Proto& proto) -> decltype(proto.s6(), void()) {
            set_vector_from_protobuf::s0(matrix.col(6), proto.s6());
            set_vector_from_protobuf::s1(matrix.col(6), proto.s6());
            set_vector_from_protobuf::s2(matrix.col(6), proto.s6());
            set_vector_from_protobuf::s3(matrix.col(6), proto.s6());
            set_vector_from_protobuf::s4(matrix.col(6), proto.s6());
            set_vector_from_protobuf::s5(matrix.col(6), proto.s6());
            set_vector_from_protobuf::s6(matrix.col(6), proto.s6());
            set_vector_from_protobuf::s7(matrix.col(6), proto.s6());
            set_vector_from_protobuf::s8(matrix.col(6), proto.s6());
            set_vector_from_protobuf::s9(matrix.col(6), proto.s6());
            set_vector_from_protobuf::sa(matrix.col(6), proto.s6());
            set_vector_from_protobuf::sb(matrix.col(6), proto.s6());
            set_vector_from_protobuf::sc(matrix.col(6), proto.s6());
            set_vector_from_protobuf::sd(matrix.col(6), proto.s6());
            set_vector_from_protobuf::se(matrix.col(6), proto.s6());
            set_vector_from_protobuf::sf(matrix.col(6), proto.s6());
        }
        template <typename... Args>
        inline void s6(const Args&...) {}

        template <typename Proto, typename Matrix>
        inline auto s7(Matrix& matrix, const Proto& proto) -> decltype(proto.s7(), void()) {
            set_vector_from_protobuf::s0(matrix.col(7), proto.s7());
            set_vector_from_protobuf::s1(matrix.col(7), proto.s7());
            set_vector_from_protobuf::s2(matrix.col(7), proto.s7());
            set_vector_from_protobuf::s3(matrix.col(7), proto.s7());
            set_vector_from_protobuf::s4(matrix.col(7), proto.s7());
            set_vector_from_protobuf::s5(matrix.col(7), proto.s7());
            set_vector_from_protobuf::s6(matrix.col(7), proto.s7());
            set_vector_from_protobuf::s7(matrix.col(7), proto.s7());
            set_vector_from_protobuf::s8(matrix.col(7), proto.s7());
            set_vector_from_protobuf::s9(matrix.col(7), proto.s7());
            set_vector_from_protobuf::sa(matrix.col(7), proto.s7());
            set_vector_from_protobuf::sb(matrix.col(7), proto.s7());
            set_vector_from_protobuf::sc(matrix.col(7), proto.s7());
            set_vector_from_protobuf::sd(matrix.col(7), proto.s7());
            set_vector_from_protobuf::se(matrix.col(7), proto.s7());
            set_vector_from_protobuf::sf(matrix.col(7), proto.s7());
        }
        template <typename... Args>
        inline void s7(const Args&...) {}

        template <typename Proto, typename Matrix>
        inline auto s8(Matrix& matrix, const Proto& proto) -> decltype(proto.s8(), void()) {
            set_vector_from_protobuf::s0(matrix.col(8), proto.s8());
            set_vector_from_protobuf::s1(matrix.col(8), proto.s8());
            set_vector_from_protobuf::s2(matrix.col(8), proto.s8());
            set_vector_from_protobuf::s3(matrix.col(8), proto.s8());
            set_vector_from_protobuf::s4(matrix.col(8), proto.s8());
            set_vector_from_protobuf::s5(matrix.col(8), proto.s8());
            set_vector_from_protobuf::s6(matrix.col(8), proto.s8());
            set_vector_from_protobuf::s7(matrix.col(8), proto.s8());
            set_vector_from_protobuf::s8(matrix.col(8), proto.s8());
            set_vector_from_protobuf::s9(matrix.col(8), proto.s8());
            set_vector_from_protobuf::sa(matrix.col(8), proto.s8());
            set_vector_from_protobuf::sb(matrix.col(8), proto.s8());
            set_vector_from_protobuf::sc(matrix.col(8), proto.s8());
            set_vector_from_protobuf::sd(matrix.col(8), proto.s8());
            set_vector_from_protobuf::se(matrix.col(8), proto.s8());
            set_vector_from_protobuf::sf(matrix.col(8), proto.s8());
        }
        template <typename... Args>
        inline void s8(const Args&...) {}

        template <typename Proto, typename Matrix>
        inline auto s9(Matrix& matrix, const Proto& proto) -> decltype(proto.s9(), void()) {
            set_vector_from_protobuf::s0(matrix.col(9), proto.s9());
            set_vector_from_protobuf::s1(matrix.col(9), proto.s9());
            set_vector_from_protobuf::s2(matrix.col(9), proto.s9());
            set_vector_from_protobuf::s3(matrix.col(9), proto.s9());
            set_vector_from_protobuf::s4(matrix.col(9), proto.s9());
            set_vector_from_protobuf::s5(matrix.col(9), proto.s9());
            set_vector_from_protobuf::s6(matrix.col(9), proto.s9());
            set_vector_from_protobuf::s7(matrix.col(9), proto.s9());
            set_vector_from_protobuf::s8(matrix.col(9), proto.s9());
            set_vector_from_protobuf::s9(matrix.col(9), proto.s9());
            set_vector_from_protobuf::sa(matrix.col(9), proto.s9());
            set_vector_from_protobuf::sb(matrix.col(9), proto.s9());
            set_vector_from_protobuf::sc(matrix.col(9), proto.s9());
            set_vector_from_protobuf::sd(matrix.col(9), proto.s9());
            set_vector_from_protobuf::se(matrix.col(9), proto.s9());
            set_vector_from_protobuf::sf(matrix.col(9), proto.s9());
        }
        template <typename... Args>
        inline void s9(const Args&...) {}

        template <typename Proto, typename Matrix>
        inline auto sa(Matrix& matrix, const Proto& proto) -> decltype(proto.sa(), void()) {
            set_vector_from_protobuf::s0(matrix.col(10), proto.sa());
            set_vector_from_protobuf::s1(matrix.col(10), proto.sa());
            set_vector_from_protobuf::s2(matrix.col(10), proto.sa());
            set_vector_from_protobuf::s3(matrix.col(10), proto.sa());
            set_vector_from_protobuf::s4(matrix.col(10), proto.sa());
            set_vector_from_protobuf::s5(matrix.col(10), proto.sa());
            set_vector_from_protobuf::s6(matrix.col(10), proto.sa());
            set_vector_from_protobuf::s7(matrix.col(10), proto.sa());
            set_vector_from_protobuf::s8(matrix.col(10), proto.sa());
            set_vector_from_protobuf::s9(matrix.col(10), proto.sa());
            set_vector_from_protobuf::sa(matrix.col(10), proto.sa());
            set_vector_from_protobuf::sb(matrix.col(10), proto.sa());
            set_vector_from_protobuf::sc(matrix.col(10), proto.sa());
            set_vector_from_protobuf::sd(matrix.col(10), proto.sa());
            set_vector_from_protobuf::se(matrix.col(10), proto.sa());
            set_vector_from_protobuf::sf(matrix.col(10), proto.sa());
        }
        template <typename... Args>
        inline void sa(const Args&...) {}

        template <typename Proto, typename Matrix>
        inline auto sb(Matrix& matrix, const Proto& proto) -> decltype(proto.sb(), void()) {
            set_vector_from_protobuf::s0(matrix.col(11), proto.sb());
            set_vector_from_protobuf::s1(matrix.col(11), proto.sb());
            set_vector_from_protobuf::s2(matrix.col(11), proto.sb());
            set_vector_from_protobuf::s3(matrix.col(11), proto.sb());
            set_vector_from_protobuf::s4(matrix.col(11), proto.sb());
            set_vector_from_protobuf::s5(matrix.col(11), proto.sb());
            set_vector_from_protobuf::s6(matrix.col(11), proto.sb());
            set_vector_from_protobuf::s7(matrix.col(11), proto.sb());
            set_vector_from_protobuf::s8(matrix.col(11), proto.sb());
            set_vector_from_protobuf::s9(matrix.col(11), proto.sb());
            set_vector_from_protobuf::sa(matrix.col(11), proto.sb());
            set_vector_from_protobuf::sb(matrix.col(11), proto.sb());
            set_vector_from_protobuf::sc(matrix.col(11), proto.sb());
            set_vector_from_protobuf::sd(matrix.col(11), proto.sb());
            set_vector_from_protobuf::se(matrix.col(11), proto.sb());
            set_vector_from_protobuf::sf(matrix.col(11), proto.sb());
        }
        template <typename... Args>
        inline void sb(const Args&...) {}

        template <typename Proto, typename Matrix>
        inline auto sc(Matrix& matrix, const Proto& proto) -> decltype(proto.sc(), void()) {
            set_vector_from_protobuf::s0(matrix.col(12), proto.sc());
            set_vector_from_protobuf::s1(matrix.col(12), proto.sc());
            set_vector_from_protobuf::s2(matrix.col(12), proto.sc());
            set_vector_from_protobuf::s3(matrix.col(12), proto.sc());
            set_vector_from_protobuf::s4(matrix.col(12), proto.sc());
            set_vector_from_protobuf::s5(matrix.col(12), proto.sc());
            set_vector_from_protobuf::s6(matrix.col(12), proto.sc());
            set_vector_from_protobuf::s7(matrix.col(12), proto.sc());
            set_vector_from_protobuf::s8(matrix.col(12), proto.sc());
            set_vector_from_protobuf::s9(matrix.col(12), proto.sc());
            set_vector_from_protobuf::sa(matrix.col(12), proto.sc());
            set_vector_from_protobuf::sb(matrix.col(12), proto.sc());
            set_vector_from_protobuf::sc(matrix.col(12), proto.sc());
            set_vector_from_protobuf::sd(matrix.col(12), proto.sc());
            set_vector_from_protobuf::se(matrix.col(12), proto.sc());
            set_vector_from_protobuf::sf(matrix.col(12), proto.sc());
        }
        template <typename... Args>
        inline void sc(const Args&...) {}

        template <typename Proto, typename Matrix>
        inline auto sd(Matrix& matrix, const Proto& proto) -> decltype(proto.sd(), void()) {
            set_vector_from_protobuf::s0(matrix.col(13), proto.sd());
            set_vector_from_protobuf::s1(matrix.col(13), proto.sd());
            set_vector_from_protobuf::s2(matrix.col(13), proto.sd());
            set_vector_from_protobuf::s3(matrix.col(13), proto.sd());
            set_vector_from_protobuf::s4(matrix.col(13), proto.sd());
            set_vector_from_protobuf::s5(matrix.col(13), proto.sd());
            set_vector_from_protobuf::s6(matrix.col(13), proto.sd());
            set_vector_from_protobuf::s7(matrix.col(13), proto.sd());
            set_vector_from_protobuf::s8(matrix.col(13), proto.sd());
            set_vector_from_protobuf::s9(matrix.col(13), proto.sd());
            set_vector_from_protobuf::sa(matrix.col(13), proto.sd());
            set_vector_from_protobuf::sb(matrix.col(13), proto.sd());
            set_vector_from_protobuf::sc(matrix.col(13), proto.sd());
            set_vector_from_protobuf::sd(matrix.col(13), proto.sd());
            set_vector_from_protobuf::se(matrix.col(13), proto.sd());
            set_vector_from_protobuf::sf(matrix.col(13), proto.sd());
        }
        template <typename... Args>
        inline void sd(const Args&...) {}

        template <typename Proto, typename Matrix>
        inline auto se(Matrix& matrix, const Proto& proto) -> decltype(proto.se(), void()) {
            set_vector_from_protobuf::s0(matrix.col(14), proto.se());
            set_vector_from_protobuf::s1(matrix.col(14), proto.se());
            set_vector_from_protobuf::s2(matrix.col(14), proto.se());
            set_vector_from_protobuf::s3(matrix.col(14), proto.se());
            set_vector_from_protobuf::s4(matrix.col(14), proto.se());
            set_vector_from_protobuf::s5(matrix.col(14), proto.se());
            set_vector_from_protobuf::s6(matrix.col(14), proto.se());
            set_vector_from_protobuf::s7(matrix.col(14), proto.se());
            set_vector_from_protobuf::s8(matrix.col(14), proto.se());
            set_vector_from_protobuf::s9(matrix.col(14), proto.se());
            set_vector_from_protobuf::sa(matrix.col(14), proto.se());
            set_vector_from_protobuf::sb(matrix.col(14), proto.se());
            set_vector_from_protobuf::sc(matrix.col(14), proto.se());
            set_vector_from_protobuf::sd(matrix.col(14), proto.se());
            set_vector_from_protobuf::se(matrix.col(14), proto.se());
            set_vector_from_protobuf::sf(matrix.col(14), proto.se());
        }
        template <typename... Args>
        inline void se(const Args&...) {}

        template <typename Proto, typename Matrix>
        inline auto sf(Matrix& matrix, const Proto& proto) -> decltype(proto.sf(), void()) {
            set_vector_from_protobuf::s0(matrix.col(15), proto.sf());
            set_vector_from_protobuf::s1(matrix.col(15), proto.sf());
            set_vector_from_protobuf::s2(matrix.col(15), proto.sf());
            set_vector_from_protobuf::s3(matrix.col(15), proto.sf());
            set_vector_from_protobuf::s4(matrix.col(15), proto.sf());
            set_vector_from_protobuf::s5(matrix.col(15), proto.sf());
            set_vector_from_protobuf::s6(matrix.col(15), proto.sf());
            set_vector_from_protobuf::s7(matrix.col(15), proto.sf());
            set_vector_from_protobuf::s8(matrix.col(15), proto.sf());
            set_vector_from_protobuf::s9(matrix.col(15), proto.sf());
            set_vector_from_protobuf::sa(matrix.col(15), proto.sf());
            set_vector_from_protobuf::sb(matrix.col(15), proto.sf());
            set_vector_from_protobuf::sc(matrix.col(15), proto.sf());
            set_vector_from_protobuf::sd(matrix.col(15), proto.sf());
            set_vector_from_protobuf::se(matrix.col(15), proto.sf());
            set_vector_from_protobuf::sf(matrix.col(15), proto.sf());
        }
        template <typename... Args>
        inline void sf(const Args&...) {}
    }  // namespace set_matrix_from_protobuf

    /*
     * Fixed sized streaming operators
     */

    template <typename Proto>
    inline Proto& convert(Proto& proto, const VecProto<Proto>& vector) {
        // vec2 - vec4
        set_protobuf_from_vector::x(proto, vector);
        set_protobuf_from_vector::y(proto, vector);
        set_protobuf_from_vector::z(proto, vector);
        set_protobuf_from_vector::t(proto, vector);

        // vec5 - vec16
        set_protobuf_from_vector::s0(proto, vector);
        set_protobuf_from_vector::s1(proto, vector);
        set_protobuf_from_vector::s2(proto, vector);
        set_protobuf_from_vector::s3(proto, vector);
        set_protobuf_from_vector::s4(proto, vector);
        set_protobuf_from_vector::s5(proto, vector);
        set_protobuf_from_vector::s6(proto, vector);
        set_protobuf_from_vector::s7(proto, vector);
        set_protobuf_from_vector::s8(proto, vector);
        set_protobuf_from_vector::s9(proto, vector);
        set_protobuf_from_vector::sa(proto, vector);
        set_protobuf_from_vector::sb(proto, vector);
        set_protobuf_from_vector::sc(proto, vector);
        set_protobuf_from_vector::sd(proto, vector);
        set_protobuf_from_vector::se(proto, vector);
        set_protobuf_from_vector::sf(proto, vector);

        return proto;
    }

    template <typename Vector>
    inline Vector& convert(Vector& vector, const ProtoVec<Vector>& proto) {
        // vec2 - vec4
        set_vector_from_protobuf::x(vector, proto);
        set_vector_from_protobuf::y(vector, proto);
        set_vector_from_protobuf::z(vector, proto);
        set_vector_from_protobuf::t(vector, proto);

        // vec5 - vec16
        set_vector_from_protobuf::s0(vector, proto);
        set_vector_from_protobuf::s1(vector, proto);
        set_vector_from_protobuf::s2(vector, proto);
        set_vector_from_protobuf::s3(vector, proto);
        set_vector_from_protobuf::s4(vector, proto);
        set_vector_from_protobuf::s5(vector, proto);
        set_vector_from_protobuf::s6(vector, proto);
        set_vector_from_protobuf::s7(vector, proto);
        set_vector_from_protobuf::s8(vector, proto);
        set_vector_from_protobuf::s9(vector, proto);
        set_vector_from_protobuf::sa(vector, proto);
        set_vector_from_protobuf::sb(vector, proto);
        set_vector_from_protobuf::sc(vector, proto);
        set_vector_from_protobuf::sd(vector, proto);
        set_vector_from_protobuf::se(vector, proto);
        set_vector_from_protobuf::sf(vector, proto);

        return vector;
    }

    template <typename Proto>
    inline Proto& convert(Proto& proto, const MatProto<Proto>& matrix) {
        // mat2 -  mat4
        set_protobuf_from_matrix::x(proto, matrix);
        set_protobuf_from_matrix::y(proto, matrix);
        set_protobuf_from_matrix::z(proto, matrix);
        set_protobuf_from_matrix::t(proto, matrix);

        // mat5 - mat16
        set_protobuf_from_matrix::s0(proto, matrix);
        set_protobuf_from_matrix::s1(proto, matrix);
        set_protobuf_from_matrix::s2(proto, matrix);
        set_protobuf_from_matrix::s3(proto, matrix);
        set_protobuf_from_matrix::s4(proto, matrix);
        set_protobuf_from_matrix::s5(proto, matrix);
        set_protobuf_from_matrix::s6(proto, matrix);
        set_protobuf_from_matrix::s7(proto, matrix);
        set_protobuf_from_matrix::s8(proto, matrix);
        set_protobuf_from_matrix::s9(proto, matrix);
        set_protobuf_from_matrix::sa(proto, matrix);
        set_protobuf_from_matrix::sb(proto, matrix);
        set_protobuf_from_matrix::sc(proto, matrix);
        set_protobuf_from_matrix::sd(proto, matrix);
        set_protobuf_from_matrix::se(proto, matrix);
        set_protobuf_from_matrix::sf(proto, matrix);

        return proto;
    }

    template <typename Matrix>
    Matrix& convert(Matrix& matrix, const ProtoMat<Matrix>& proto) {
        // mat2 - mat4
        set_matrix_from_protobuf::x(matrix, proto);
        set_matrix_from_protobuf::y(matrix, proto);
        set_matrix_from_protobuf::z(matrix, proto);
        set_matrix_from_protobuf::t(matrix, proto);

        // mat5 - mat16
        set_matrix_from_protobuf::s0(matrix, proto);
        set_matrix_from_protobuf::s1(matrix, proto);
        set_matrix_from_protobuf::s2(matrix, proto);
        set_matrix_from_protobuf::s3(matrix, proto);
        set_matrix_from_protobuf::s4(matrix, proto);
        set_matrix_from_protobuf::s5(matrix, proto);
        set_matrix_from_protobuf::s6(matrix, proto);
        set_matrix_from_protobuf::s7(matrix, proto);
        set_matrix_from_protobuf::s8(matrix, proto);
        set_matrix_from_protobuf::s9(matrix, proto);
        set_matrix_from_protobuf::sa(matrix, proto);
        set_matrix_from_protobuf::sb(matrix, proto);
        set_matrix_from_protobuf::sc(matrix, proto);
        set_matrix_from_protobuf::sd(matrix, proto);
        set_matrix_from_protobuf::se(matrix, proto);
        set_matrix_from_protobuf::sf(matrix, proto);

        return matrix;
    }

    /*
     * Dynamic sized streaming operators
     */

    template <typename Proto>
    inline Proto& convert(Proto& proto, const DynamicVecProto<Proto> vector) {

        // Reserve enough space
        proto.mutable_v()->Reserve(vector.size());

        // Populate the data
        for (int i = 0; i < vector.size(); ++i) {
            proto.add_v(vector[i]);
        }
        return proto;
    }
    template <typename Vector>
    inline Vector& convert(Vector& vector, const DynamicProtoVec<Vector> proto) {

        // Reserve enough space
        vector.resize(proto.v_size());

        // Populate the data
        for (int i = 0; i < proto.v_size(); ++i) {
            vector[i] = proto.v(i);
        }
        return vector;
    }

    template <typename Proto>
    inline Proto& convert(Proto& proto, const DynamicMatProto<Proto> matrix) {

        // Set our rows and columns
        proto.set_rows(matrix.rows());
        proto.set_cols(matrix.cols());

        // Allocate the memory
        proto.mutable_v()->Resize(matrix.size(), 0);

        // Copy over
        Eigen::Map<DynamicMatProto<Proto>>(
            const_cast<double*>(proto.mutable_v()->data()), matrix.rows(), matrix.cols()) = matrix;

        return proto;
    }
    template <typename Matrix>
    inline Matrix& convert(Matrix& matrix, const DynamicProtoMat<Matrix> proto) {

        // Copy the data over
        matrix = Eigen::Map<const Matrix>(proto.v().data(), proto.rows(), proto.cols());

        return matrix;
    }

    inline ::message::conversion::math::cvec& convert(::message::conversion::math::cvec& vector, const ::cvec& proto) {

        vector = Eigen::Map<const ::message::conversion::math::cvec>(reinterpret_cast<const uint8_t*>(proto.v().data()),
                                                                     proto.v().size());

        return vector;
    }

    inline ::cvec& convert(::cvec& proto, const ::message::conversion::math::cvec& vector) {

        proto.mutable_v()->resize(vector.size());

        // Copy the data across
        Eigen::Map<::message::conversion::math::cvec>(
            reinterpret_cast<uint8_t*>(const_cast<char*>(proto.mutable_v()->data())), proto.v().size()) = vector;

        return proto;
    }

    inline ::message::conversion::math::cmat& convert(::message::conversion::math::cmat& matrix, const ::cmat& proto) {

        // Map the data and copy it across
        matrix = Eigen::Map<const ::message::conversion::math::cmat>(
            reinterpret_cast<const uint8_t*>(proto.v().data()), proto.rows(), proto.cols());

        return matrix;
    }

    inline ::cmat& convert(::cmat& proto, const ::message::conversion::math::cmat& matrix) {

        // Set our size
        proto.set_rows(matrix.rows());
        proto.set_cols(matrix.cols());

        // Allocate space
        proto.mutable_v()->resize(matrix.size());

        // Copy it across
        Eigen::Map<::message::conversion::math::cmat>(
            reinterpret_cast<uint8_t*>(const_cast<char*>(proto.mutable_v()->data())), matrix.rows(), matrix.cols()) =
            matrix;

        return proto;
    }

}  // namespace conversion
}  // namespace message

#endif  // MESSAGE_CONVERSION_PROTO_MATRIX_H
