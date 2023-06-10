#ifndef MESSAGE_CONVERSION_PROTO_CONVERSION_HPP
#define MESSAGE_CONVERSION_PROTO_CONVERSION_HPP

#include <chrono>
#include <google/protobuf/duration.pb.h>
#include <google/protobuf/timestamp.pb.h>
#include <nuclear_bits/clock.hpp>

#include "neutron_type_map.hpp"
#include "proto_neutron_map.hpp"
#include "proto_neutron_sfinae.hpp"

namespace message::conversion {
    template <typename Type, typename Neutron, typename Proto>
    struct Convert {};

    /**
     * @brief Specialisation for converting to/from fixed sized vector types
     *
     * @tparam Neutron Expected to be one of Eigen::Matrix<Scalar, 2+, 1> or Eigen::Matrix<Scalar, 1, 2+>
     * @tparam Proto Expected to be one of ::(|f|u|i)vec(2-16)
     */
    template <typename Neutron, typename Proto>
    struct Convert<Vector, Neutron, Proto> {
        static Neutron call(const Proto& proto) {
            Neutron vector{};

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
        static Proto call(const Neutron& neutron) {
            Proto proto{};

            // vec2 - vec4
            set_protobuf_from_vector::x(proto, neutron);
            set_protobuf_from_vector::y(proto, neutron);
            set_protobuf_from_vector::z(proto, neutron);
            set_protobuf_from_vector::t(proto, neutron);

            // vec5 - vec16
            set_protobuf_from_vector::s0(proto, neutron);
            set_protobuf_from_vector::s1(proto, neutron);
            set_protobuf_from_vector::s2(proto, neutron);
            set_protobuf_from_vector::s3(proto, neutron);
            set_protobuf_from_vector::s4(proto, neutron);
            set_protobuf_from_vector::s5(proto, neutron);
            set_protobuf_from_vector::s6(proto, neutron);
            set_protobuf_from_vector::s7(proto, neutron);
            set_protobuf_from_vector::s8(proto, neutron);
            set_protobuf_from_vector::s9(proto, neutron);
            set_protobuf_from_vector::sa(proto, neutron);
            set_protobuf_from_vector::sb(proto, neutron);
            set_protobuf_from_vector::sc(proto, neutron);
            set_protobuf_from_vector::sd(proto, neutron);
            set_protobuf_from_vector::se(proto, neutron);
            set_protobuf_from_vector::sf(proto, neutron);

            return proto;
        }
    };

    /**
     * @brief Specialisation for converting to/from fixed sized matrix types
     *
     * @tparam Neutron Expected to be one of Eigen::Matrix<Scalar, 2+, 2+> or Eigen::Matrix<Scalar, 2+, 2+>
     * @tparam Proto Expected to be one of ::(|f|u|i)mat(2-16)
     */
    template <typename Neutron, typename Proto>
    struct Convert<Matrix, Neutron, Proto> {
        static Neutron call(const Proto& proto) {
            Neutron matrix{};

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
        static Proto call(const Neutron& neutron) {
            Proto proto{};

            // mat2 -  mat4
            set_protobuf_from_matrix::x(proto, neutron);
            set_protobuf_from_matrix::y(proto, neutron);
            set_protobuf_from_matrix::z(proto, neutron);
            set_protobuf_from_matrix::t(proto, neutron);

            // mat5 - mat16
            set_protobuf_from_matrix::s0(proto, neutron);
            set_protobuf_from_matrix::s1(proto, neutron);
            set_protobuf_from_matrix::s2(proto, neutron);
            set_protobuf_from_matrix::s3(proto, neutron);
            set_protobuf_from_matrix::s4(proto, neutron);
            set_protobuf_from_matrix::s5(proto, neutron);
            set_protobuf_from_matrix::s6(proto, neutron);
            set_protobuf_from_matrix::s7(proto, neutron);
            set_protobuf_from_matrix::s8(proto, neutron);
            set_protobuf_from_matrix::s9(proto, neutron);
            set_protobuf_from_matrix::sa(proto, neutron);
            set_protobuf_from_matrix::sb(proto, neutron);
            set_protobuf_from_matrix::sc(proto, neutron);
            set_protobuf_from_matrix::sd(proto, neutron);
            set_protobuf_from_matrix::se(proto, neutron);
            set_protobuf_from_matrix::sf(proto, neutron);

            return proto;
        }
    };

    /**
     * @brief Specialisation for converting to/from dynamic sized vector types
     *
     * @tparam Neutron Expected to be one of Eigen::Matrix<Scalar, Eigen::Dynamic, 1> or Eigen::Matrix<Scalar, 1,
     * Eigen::Dynamic>
     * @tparam Proto Expected to be one of ::(|f|u|i)vec
     */
    template <typename Neutron, typename Proto>
    struct Convert<DynamicVector, Neutron, Proto> {
        static Neutron call(const Proto& proto) {
            Neutron vector{};

            // Reserve enough space
            vector.resize(proto.v_size());

            // Copy across
            vector = Eigen::Map<const Neutron>(proto.v().data(), proto.v_size());

            return vector;
        }
        static Proto call(const Neutron& neutron) {
            Proto proto{};

            // Reserve enough space
            proto.mutable_v()->Resize(neutron.size(), 0);

            // Copy across
            Eigen::Map<Neutron>(
                // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
                const_cast<typename Neutron::Scalar*>(proto.mutable_v()->data()),
                neutron.size()) = neutron;

            return proto;
        }
    };

    /**
     * @brief Specialisation for converting to/from cvec types
     *
     * @details cvec is a dynamic sized vector with a scalar type of char. This specialisation exists as the Protobuf
     * types uses a signed char but the Neutron expects an unsigned char.
     */
    template <>
    struct Convert<DynamicVector, ::message::conversion::math::cvec, ::cvec> {
        static ::message::conversion::math::cvec call(const ::cvec& proto) {
            ::message::conversion::math::cvec vector =
                Eigen::Map<const ::message::conversion::math::cvec>(reinterpret_cast<const uint8_t*>(proto.v().data()),
                                                                    Eigen::Index(proto.v().size()));

            return vector;
        }
        static ::cvec call(const ::message::conversion::math::cvec& vector) {
            ::cvec proto{};

            proto.mutable_v()->resize(vector.size());

            // Copy the data across
            Eigen::Map<::message::conversion::math::cvec>(reinterpret_cast<uint8_t*>(proto.mutable_v()->data()),
                                                          Eigen::Index(proto.v().size())) = vector;

            return proto;
        }
    };

    /**
     * @brief Specialisation for converting to/from dynamic sized matrix types
     *
     * @tparam Neutron Expected to be Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>
     * @tparam Proto Expected to be one of ::(|f|u|i)mat
     */
    template <typename Neutron, typename Proto>
    struct Convert<DynamicMatrix, Neutron, Proto> {
        static Neutron call(const Proto& proto) {
            Neutron matrix{};

            // Copy the data over
            matrix = Eigen::Map<const Neutron>(proto.v().data(), proto.rows(), proto.cols());

            return matrix;
        }
        static Proto call(const Neutron& neutron) {
            Proto proto{};

            // Set our rows and columns
            proto.set_rows(neutron.rows());
            proto.set_cols(neutron.cols());

            // Allocate the memory
            proto.mutable_v()->Resize(neutron.size(), 0);

            // Copy over
            Eigen::Map<Neutron>(
                // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
                const_cast<typename Neutron::Scalar*>(proto.mutable_v()->data()),
                neutron.rows(),
                neutron.cols()) = neutron;

            return proto;
        }
    };

    /**
     * @brief Specialisation for converting to/from cmat types
     *
     * @details cmat is a dynamic sized matrix with a scalar type of char. This specialisation exists as the Protobuf
     * types uses a signed char but the Neutron expects an unsigned char.
     */
    template <>
    struct Convert<DynamicMatrix, ::message::conversion::math::cmat, ::cmat> {
        static ::message::conversion::math::cmat call(const ::cmat& proto) {

            // Map the data and copy it across
            ::message::conversion::math::cmat matrix =
                Eigen::Map<const ::message::conversion::math::cmat>(reinterpret_cast<const uint8_t*>(proto.v().data()),
                                                                    proto.rows(),
                                                                    proto.cols());

            return matrix;
        }
        static ::cmat call(const ::message::conversion::math::cmat& matrix) {
            ::cmat proto{};

            // Set our size
            proto.set_rows(matrix.rows());
            proto.set_cols(matrix.cols());

            // Allocate space
            proto.mutable_v()->resize(matrix.size());

            // Copy it across
            Eigen::Map<::message::conversion::math::cmat>(reinterpret_cast<uint8_t*>(proto.mutable_v()->data()),
                                                          matrix.rows(),
                                                          matrix.cols()) = matrix;

            return proto;
        }
    };

    /**
     * @brief Specialisation for converting to/from Eigen::Isometry types
     *
     * @tparam Neutron Expected to be Eigen::Isometry<....>
     * @tparam Proto Expected to be one of ::iso2/3 of ::fiso2/3
     */
    template <typename Neutron, typename Proto>
    struct Convert<Isometry, Neutron, Proto> {
        static Neutron call(const Proto& proto) {
            Neutron isometry{};

            set_matrix_from_protobuf::x(isometry.matrix(), proto);
            set_matrix_from_protobuf::y(isometry.matrix(), proto);
            set_matrix_from_protobuf::z(isometry.matrix(), proto);
            set_matrix_from_protobuf::t(isometry.matrix(), proto);

            return isometry;
        }
        static Proto call(const Neutron& neutron) {
            Proto proto{};

            set_protobuf_from_matrix::x(proto, neutron.matrix());
            set_protobuf_from_matrix::y(proto, neutron.matrix());
            set_protobuf_from_matrix::z(proto, neutron.matrix());
            set_protobuf_from_matrix::t(proto, neutron.matrix());

            return proto;
        }
    };

    /**
     * @brief Specialisation for converting to/from Eigen::Quaternion types
     *
     * @tparam Neutron Expected to be Eigen::Quaternion<....>
     * @tparam Proto Expected to be one of ::quat of ::fquat
     */
    template <typename Neutron, typename Proto>
    struct Convert<Quaternion, Neutron, Proto> {
        static Neutron call(const Proto& proto) {
            Neutron quaternion{};

            quaternion.x() = proto.x();
            quaternion.y() = proto.y();
            quaternion.z() = proto.z();
            quaternion.w() = proto.w();

            return quaternion;
        }
        static Proto call(const Neutron& neutron) {
            Proto proto{};

            proto.set_x(neutron.x());
            proto.set_y(neutron.y());
            proto.set_z(neutron.z());
            proto.set_w(neutron.w());

            return proto;
        }
    };

    /**
     * @brief Specialisation for converting to/from chrono types
     *
     * @tparam Neutron Expected to be one of NUClear::clock::time_point or NUClear::clock::duration
     * @tparam Proto Expected to be one of ::google::protobuf::Timestamp or ::google::protobuf::Duration
     */
    template <typename Neutron, typename Proto>
    struct Convert<Chrono, Neutron, Proto> {
        static NUClear::clock::time_point call(const ::google::protobuf::Timestamp& proto) {
            NUClear::clock::time_point t{};

            // Get our seconds and nanos in c++ land
            auto seconds = std::chrono::seconds(proto.seconds());
            auto nanos   = std::chrono::nanoseconds(proto.nanos());

            // Make a timestamp out of the summation of them
            t = NUClear::clock::time_point(seconds + nanos);

            return t;
        }
        static ::google::protobuf::Timestamp call(const NUClear::clock::time_point& t) {
            ::google::protobuf::Timestamp proto{};

            // Get the epoch timestamp
            auto d = t.time_since_epoch();

            // Get our seconds and the remainder nanoseconds
            auto seconds = std::chrono::duration_cast<std::chrono::seconds>(d);
            auto nanos   = std::chrono::duration_cast<std::chrono::nanoseconds>(d - seconds);

            // Set our seconds and nanoseconds
            proto.set_seconds(seconds.count());
            proto.set_nanos(int32_t(nanos.count()));

            return proto;
        }
        static NUClear::clock::duration call(const ::google::protobuf::Duration& proto) {
            NUClear::clock::duration d{};

            // Get our seconds and nanos in c++ land
            auto seconds = std::chrono::seconds(proto.seconds());
            auto nanos   = std::chrono::nanoseconds(proto.nanos());

            // Make a duration out of the summation of them
            d = std::chrono::duration_cast<NUClear::clock::duration>(seconds + nanos);

            return d;
        }
        static ::google::protobuf::Duration call(const NUClear::clock::duration& d) {
            ::google::protobuf::Duration proto{};

            // Get our seconds and the remainder nanoseconds
            auto seconds = std::chrono::duration_cast<std::chrono::seconds>(d);
            auto nanos   = std::chrono::duration_cast<std::chrono::nanoseconds>(d - seconds);

            // Set our seconds and nanoseconds
            proto.set_seconds(seconds.count());
            proto.set_nanos(int32_t(nanos.count()));

            return proto;
        }
    };

    /**
     * @brief Converts a Neutron type into a Protobuf type
     *
     * @tparam Neutron The Neutron type to convert from
     *
     * @param proto The Neutron instance to convert from
     */
    template <typename Neutron>
    inline ProtoNeutron<Neutron> convert(const Neutron& neutron) {
        return Convert<NeutronType<Neutron>, Neutron, ProtoNeutron<Neutron>>::call(neutron);
    }

    /**
     * @brief Converts a Protobuf type into a Neutron type
     *
     * @tparam Neutron The Neutron type to convert to
     *
     * @param proto The Protobuf instance to convert from
     */
    template <typename Neutron>
    inline Neutron convert(const ProtoNeutron<Neutron>& proto) {
        return Convert<NeutronType<Neutron>, Neutron, ProtoNeutron<Neutron>>::call(proto);
    }

}  // namespace message::conversion

#endif  // MESSAGE_CONVERSION_PROTO_CONVERSION_HPP
