#ifndef MESSAGE_CONVERSION_PROTO_MATRIX_H
#define MESSAGE_CONVERSION_PROTO_MATRIX_H

#include "math_types.h"
#include "Vector.pb.h"
#include "Matrix.pb.h"
/**
 * @brief This type maps protocol buffer types to their vector equivalents
 * @tparam T the protocol buffer type to map
 */
template <typename T>
struct ProtobufToVectorMap;
template <> struct ProtobufToVectorMap<::vec2>   { using type = ::message::conversion::math::vec2;   };
template <> struct ProtobufToVectorMap<::vec3>   { using type = ::message::conversion::math::vec3;   };
template <> struct ProtobufToVectorMap<::vec4>   { using type = ::message::conversion::math::vec4;   };
template <> struct ProtobufToVectorMap<::fvec2>  { using type = ::message::conversion::math::fvec2;  };
template <> struct ProtobufToVectorMap<::fvec3>  { using type = ::message::conversion::math::fvec3;  };
template <> struct ProtobufToVectorMap<::fvec4>  { using type = ::message::conversion::math::fvec4;  };
template <> struct ProtobufToVectorMap<::ivec2>  { using type = ::message::conversion::math::ivec2;  };
template <> struct ProtobufToVectorMap<::ivec3>  { using type = ::message::conversion::math::ivec3;  };
template <> struct ProtobufToVectorMap<::ivec4>  { using type = ::message::conversion::math::ivec4;  };
template <> struct ProtobufToVectorMap<::uvec2>  { using type = ::message::conversion::math::uvec2;  };
template <> struct ProtobufToVectorMap<::uvec3>  { using type = ::message::conversion::math::uvec3;  };
template <> struct ProtobufToVectorMap<::uvec4>  { using type = ::message::conversion::math::uvec4;  };
template <typename T>
using VecProto = typename ProtobufToVectorMap<T>::type;

template <typename T>
struct DynamicProtobufToVectorMap;
template <> struct ProtobufToVectorMap<::vec>   { using type = ::message::conversion::math::vec;   };
template <> struct ProtobufToVectorMap<::fvec>  { using type = ::message::conversion::math::fvec;  };
template <> struct ProtobufToVectorMap<::ivec>  { using type = ::message::conversion::math::ivec;  };
template <> struct ProtobufToVectorMap<::uvec>  { using type = ::message::conversion::math::uvec;  };
template <> struct ProtobufToVectorMap<::cvec>  { using type = ::message::conversion::math::cvec;  };
template <typename T>
using DynamicVecProto = typename DynamicProtobufToVectorMap<T>::type;

/**
 * @brief This type maps vector types to their protocol buffer equivalents
 *
 * @tparam T the vector type to map
 */
template <typename T>
struct VectorToProtobufMap;
template <> struct VectorToProtobufMap<::message::conversion::math::vec2>   { using type = ::vec2;   };
template <> struct VectorToProtobufMap<::message::conversion::math::vec3>   { using type = ::vec3;   };
template <> struct VectorToProtobufMap<::message::conversion::math::vec4>   { using type = ::vec4;   };
template <> struct VectorToProtobufMap<::message::conversion::math::fvec2>  { using type = ::fvec2;  };
template <> struct VectorToProtobufMap<::message::conversion::math::fvec3>  { using type = ::fvec3;  };
template <> struct VectorToProtobufMap<::message::conversion::math::fvec4>  { using type = ::fvec4;  };
template <> struct VectorToProtobufMap<::message::conversion::math::ivec2>  { using type = ::ivec2;  };
template <> struct VectorToProtobufMap<::message::conversion::math::ivec3>  { using type = ::ivec3;  };
template <> struct VectorToProtobufMap<::message::conversion::math::ivec4>  { using type = ::ivec4;  };
template <> struct VectorToProtobufMap<::message::conversion::math::uvec2>  { using type = ::uvec2;  };
template <> struct VectorToProtobufMap<::message::conversion::math::uvec3>  { using type = ::uvec3;  };
template <> struct VectorToProtobufMap<::message::conversion::math::uvec4>  { using type = ::uvec4;  };
template <typename T>
using ProtoVec = typename VectorToProtobufMap<T>::type;

template <typename T>
struct DynamicVectorToProtobufMap;
template <> struct DynamicVectorToProtobufMap<::message::conversion::math::vec>   { using type = ::vec;   };
template <> struct DynamicVectorToProtobufMap<::message::conversion::math::fvec>  { using type = ::fvec;  };
template <> struct DynamicVectorToProtobufMap<::message::conversion::math::ivec>  { using type = ::ivec;  };
template <> struct DynamicVectorToProtobufMap<::message::conversion::math::uvec>  { using type = ::uvec;  };
template <> struct DynamicVectorToProtobufMap<::message::conversion::math::cvec>  { using type = ::cvec;  };
template <typename T>
using DynamicProtoVec = typename DynamicVectorToProtobufMap<T>::type;

/**
 * @brief This type maps protocol buffer types to their matrix equivalents
 *
 * @tparam T the protocol buffer type to map
 */
template <typename T>
struct ProtobufToMatrixMap;
template <> struct ProtobufToMatrixMap<::mat22>  { using type = ::message::conversion::math::mat22;  };
template <> struct ProtobufToMatrixMap<::mat33>  { using type = ::message::conversion::math::mat33;  };
template <> struct ProtobufToMatrixMap<::mat44>  { using type = ::message::conversion::math::mat44;  };
template <> struct ProtobufToMatrixMap<::fmat22> { using type = ::message::conversion::math::fmat22; };
template <> struct ProtobufToMatrixMap<::fmat33> { using type = ::message::conversion::math::fmat33; };
template <> struct ProtobufToMatrixMap<::fmat44> { using type = ::message::conversion::math::fmat44; };
template <> struct ProtobufToMatrixMap<::imat22> { using type = ::message::conversion::math::imat22; };
template <> struct ProtobufToMatrixMap<::imat33> { using type = ::message::conversion::math::imat33; };
template <> struct ProtobufToMatrixMap<::imat44> { using type = ::message::conversion::math::imat44; };
template <> struct ProtobufToMatrixMap<::umat22> { using type = ::message::conversion::math::umat22; };
template <> struct ProtobufToMatrixMap<::umat33> { using type = ::message::conversion::math::umat33; };
template <> struct ProtobufToMatrixMap<::umat44> { using type = ::message::conversion::math::umat44; };
template <typename T>
using MatProto = typename ProtobufToMatrixMap<T>::type;

template <typename T>
struct DynamicProtobufToMatrixMap;
template <> struct DynamicProtobufToMatrixMap<::mat>  { using type = ::message::conversion::math::mat;  };
template <> struct DynamicProtobufToMatrixMap<::fmat> { using type = ::message::conversion::math::fmat; };
template <> struct DynamicProtobufToMatrixMap<::imat> { using type = ::message::conversion::math::imat; };
template <> struct DynamicProtobufToMatrixMap<::umat> { using type = ::message::conversion::math::umat; };
template <> struct DynamicProtobufToMatrixMap<::cmat> { using type = ::message::conversion::math::cmat; };
template <typename T>
using DynamicMatProto = typename DynamicProtobufToMatrixMap<T>::type;

/**
 * @brief This type maps matrix types to their protocol buffer equivalents
 *
 * @tparam T the matrix type to map
 */
template <typename T>
struct MatrixToProtobufMap;
template <> struct MatrixToProtobufMap<::message::conversion::math::mat22>  { using type = ::mat22;  };
template <> struct MatrixToProtobufMap<::message::conversion::math::mat33>  { using type = ::mat33;  };
template <> struct MatrixToProtobufMap<::message::conversion::math::mat44>  { using type = ::mat44;  };
template <> struct MatrixToProtobufMap<::message::conversion::math::fmat22> { using type = ::fmat22; };
template <> struct MatrixToProtobufMap<::message::conversion::math::fmat33> { using type = ::fmat33; };
template <> struct MatrixToProtobufMap<::message::conversion::math::fmat44> { using type = ::fmat44; };
template <> struct MatrixToProtobufMap<::message::conversion::math::imat22> { using type = ::imat22; };
template <> struct MatrixToProtobufMap<::message::conversion::math::imat33> { using type = ::imat33; };
template <> struct MatrixToProtobufMap<::message::conversion::math::imat44> { using type = ::imat44; };
template <> struct MatrixToProtobufMap<::message::conversion::math::umat22> { using type = ::umat22; };
template <> struct MatrixToProtobufMap<::message::conversion::math::umat33> { using type = ::umat33; };
template <> struct MatrixToProtobufMap<::message::conversion::math::umat44> { using type = ::umat44; };
template <typename T>
using ProtoMat = typename MatrixToProtobufMap<T>::type;

template <typename T>
struct DynamicMatrixToProtobufMap;
template <> struct DynamicMatrixToProtobufMap<::message::conversion::math::mat>  { using type = ::mat;  };
template <> struct DynamicMatrixToProtobufMap<::message::conversion::math::fmat> { using type = ::fmat; };
template <> struct DynamicMatrixToProtobufMap<::message::conversion::math::imat> { using type = ::imat; };
template <> struct DynamicMatrixToProtobufMap<::message::conversion::math::umat> { using type = ::umat; };
template <> struct DynamicMatrixToProtobufMap<::message::conversion::math::cmat> { using type = ::cmat; };
template <typename T>
using DynamicProtoMat = typename DynamicMatrixToProtobufMap<T>::type;

/**
 * @brief SFINAE functions to set protocol buffer values from vectors
 */
namespace set_protobuf_from_vector {
    template <typename Proto, typename Vector>
    inline auto x(Proto& proto, const Vector& vector) -> decltype(proto.x(), void()) {
        proto.set_x(vector[0]);
    }
    template <typename... Args>
    inline void x(const Args&...) {}

    template <typename Proto, typename Vector>
    inline auto y(Proto& proto, const Vector& vector) -> decltype(proto.y(), void()) {
        proto.set_y(vector[1]);
    }
    template <typename... Args>
    inline void y(const Args&...) {}

    template <typename Proto, typename Vector>
    inline auto z(Proto& proto, const Vector& vector) -> decltype(proto.z(), void()) {
        proto.set_z(vector[2]);
    }
    template <typename... Args>
    inline void z(const Args&...) {}

    template <typename Proto, typename Vector>
    inline auto t(Proto& proto, const Vector& vector) -> decltype(proto.t(), void()) {
        proto.set_t(vector[3]);
    }
    template <typename... Args>
    inline void t(const Args&...) {}
}

/**
 * @brief SFINAE functions to set vector values from protcol buffers
 */
namespace set_vector_from_protobuf {
    template <typename Proto, typename Vector>
    inline auto x(Vector&& vector, const Proto& proto) -> decltype(proto.x(), void()) {
        vector[0] = proto.x();
    }
    template <typename... Args>
    inline void x(const Args&...) {}

    template <typename Proto, typename Vector>
    inline auto y(Vector&& vector, const Proto& proto) -> decltype(proto.y(), void()) {
        vector[1] = proto.y();
    }
    template <typename... Args>
    inline void y(const Args&...) {}

    template <typename Proto, typename Vector>
    inline auto z(Vector&& vector, const Proto& proto) -> decltype(proto.z(), void()) {
        vector[2] = proto.z();
    }
    template <typename... Args>
    inline void z(const Args&...) {}

    template <typename Proto, typename Vector>
    inline auto t(Vector&& vector, const Proto& proto) -> decltype(proto.t(), void()) {
        vector[3] = proto.t();
    }
    template <typename... Args>
    inline void t(const Args&...) {}
}

/**
 * @brief SFINAE functions to set protocol buffer values
 */
namespace set_protobuf_from_matrix {
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
}

/**
 * @brief SFINAE functions to set matrix values
 */
namespace set_matrix_from_protobuf {
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
}

/*
 * Fixed sized streaming operators
 */

template <typename Proto>
inline Proto& operator<< (Proto& proto, const VecProto<Proto>& vector) {
    set_protobuf_from_vector::x(proto, vector);
    set_protobuf_from_vector::y(proto, vector);
    set_protobuf_from_vector::z(proto, vector);
    set_protobuf_from_vector::t(proto, vector);
    return proto;
}

template <typename Vector>
inline Vector& operator<< (Vector& vector, const ProtoVec<Vector>& proto) {
    set_vector_from_protobuf::x(vector, proto);
    set_vector_from_protobuf::y(vector, proto);
    set_vector_from_protobuf::z(vector, proto);
    set_vector_from_protobuf::t(vector, proto);
    return vector;
}

template <typename Proto>
inline Proto& operator<< (Proto& proto, const MatProto<Proto>& matrix) {
    set_protobuf_from_matrix::x(proto, matrix);
    set_protobuf_from_matrix::y(proto, matrix);
    set_protobuf_from_matrix::z(proto, matrix);
    set_protobuf_from_matrix::t(proto, matrix);
    return proto;
}

template <typename Matrix>
Matrix& operator<< (Matrix& matrix, const ProtoMat<Matrix>& proto) {
    set_matrix_from_protobuf::x(matrix, proto);
    set_matrix_from_protobuf::y(matrix, proto);
    set_matrix_from_protobuf::z(matrix, proto);
    set_matrix_from_protobuf::t(matrix, proto);
    return matrix;
}

/*
 * Dynamic sized streaming operators
 */

template <typename Proto>
inline Proto& operator<< (Proto& proto, const DynamicVecProto<Proto> vector) {
    proto.Reserve(vector.size());
    for(int i = 0; i < vector.size(); ++i) {
        proto.add_value(vector[i]);
    }
    return proto;
}
template <typename Vector>
inline Vector& operator<< (Vector& vector, const DynamicProtoVec<Vector> proto) {
    vector.resize(proto.v_size());
    for (int i = 0; i < proto.v_size(); ++i) {
        vector[i] = proto.v(i);
    }
    return vector;
}

template <typename Proto>
inline Proto& operator<< (Proto& proto, const DynamicMatProto<Proto> matrix) {

    // Set our rows and columns
    proto.set_rows(matrix.rows());
    proto.set_cols(matrix.cols());

    // Copy the data over
    *proto.mutable_v() = std::remove_reference_t<decltype(*proto.mutable_v())>(matrix.data(), matrix.data() + matrix.size());

    return proto;
}
template <typename Matrix>
inline Matrix& operator<< (Matrix& matrix, const DynamicProtoMat<Matrix> proto) {

    // Resize our matrix to the correct size
    matrix.resize(proto.rows(), proto.cols());

    // Copy the data over
    std::memcpy(matrix.data(), proto.v().data(), proto.v().size());

    return matrix;
}

#endif  // MESSAGE_CONVERSION_PROTO_MATRIX_H
