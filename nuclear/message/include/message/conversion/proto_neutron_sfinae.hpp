/*
 * MIT License
 *
 * Copyright (c) 2016 NUbots
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
#ifndef MESSAGE_CONVERSION_PROTO_NEUTRON_SFINAE_HPP
#define MESSAGE_CONVERSION_PROTO_NEUTRON_SFINAE_HPP

namespace message::conversion {
    /**
     * @brief SFINAE functions to set protocol buffer values from vectors
     */
    namespace set_protobuf_from_vector {
        // clang-format off
        template <typename Proto, typename Vector> inline auto x(Proto& proto, const Vector& vector) -> decltype(proto.x(), void()) { proto.set_x(vector[0]); }
        template <typename... Args> inline void x(const Args&... /*unused*/) {}
        template <typename Proto, typename Vector> inline auto y(Proto& proto, const Vector& vector) -> decltype(proto.y(), void()) { proto.set_y(vector[1]); }
        template <typename... Args> inline void y(const Args&... /*unused*/) {}
        template <typename Proto, typename Vector> inline auto z(Proto& proto, const Vector& vector) -> decltype(proto.z(), void()) { proto.set_z(vector[2]); }
        template <typename... Args> inline void z(const Args&... /*unused*/) {}
        template <typename Proto, typename Vector> inline auto t(Proto& proto, const Vector& vector) -> decltype(proto.t(), void()) { proto.set_t(vector[3]); }
        template <typename... Args> inline void t(const Args&... /*unused*/) {}

        template <typename Proto, typename Vector> inline auto s0(Proto& proto, const Vector& vector) -> decltype(proto.s0(), void()) { proto.set_s0(vector[0]); }
        template <typename... Args> inline void s0(const Args&... /*unused*/) {}
        template <typename Proto, typename Vector> inline auto s1(Proto& proto, const Vector& vector) -> decltype(proto.s1(), void()) { proto.set_s1(vector[1]); }
        template <typename... Args> inline void s1(const Args&... /*unused*/) {}
        template <typename Proto, typename Vector> inline auto s2(Proto& proto, const Vector& vector) -> decltype(proto.s2(), void()) { proto.set_s2(vector[2]); }
        template <typename... Args> inline void s2(const Args&... /*unused*/) {}
        template <typename Proto, typename Vector> inline auto s3(Proto& proto, const Vector& vector) -> decltype(proto.s3(), void()) { proto.set_s3(vector[3]); }
        template <typename... Args> inline void s3(const Args&... /*unused*/) {}
        template <typename Proto, typename Vector> inline auto s4(Proto& proto, const Vector& vector) -> decltype(proto.s4(), void()) { proto.set_s4(vector[4]); }
        template <typename... Args> inline void s4(const Args&... /*unused*/) {}
        template <typename Proto, typename Vector> inline auto s5(Proto& proto, const Vector& vector) -> decltype(proto.s5(), void()) { proto.set_s5(vector[5]); }
        template <typename... Args> inline void s5(const Args&... /*unused*/) {}
        template <typename Proto, typename Vector> inline auto s6(Proto& proto, const Vector& vector) -> decltype(proto.s6(), void()) { proto.set_s6(vector[6]); }
        template <typename... Args> inline void s6(const Args&... /*unused*/) {}
        template <typename Proto, typename Vector> inline auto s7(Proto& proto, const Vector& vector) -> decltype(proto.s7(), void()) { proto.set_s7(vector[7]); }
        template <typename... Args> inline void s7(const Args&... /*unused*/) {}
        template <typename Proto, typename Vector> inline auto s8(Proto& proto, const Vector& vector) -> decltype(proto.s8(), void()) { proto.set_s8(vector[8]); }
        template <typename... Args> inline void s8(const Args&... /*unused*/) {}
        template <typename Proto, typename Vector> inline auto s9(Proto& proto, const Vector& vector) -> decltype(proto.s9(), void()) { proto.set_s9(vector[9]); }
        template <typename... Args> inline void s9(const Args&... /*unused*/) {}
        template <typename Proto, typename Vector> inline auto sa(Proto& proto, const Vector& vector) -> decltype(proto.sa(), void()) { proto.set_sa(vector[10]); }
        template <typename... Args> inline void sa(const Args&... /*unused*/) {}
        template <typename Proto, typename Vector> inline auto sb(Proto& proto, const Vector& vector) -> decltype(proto.sb(), void()) { proto.set_sb(vector[11]); }
        template <typename... Args> inline void sb(const Args&... /*unused*/) {}
        template <typename Proto, typename Vector> inline auto sc(Proto& proto, const Vector& vector) -> decltype(proto.sc(), void()) { proto.set_sc(vector[12]); }
        template <typename... Args> inline void sc(const Args&... /*unused*/) {}
        template <typename Proto, typename Vector> inline auto sd(Proto& proto, const Vector& vector) -> decltype(proto.sd(), void()) { proto.set_sd(vector[13]); }
        template <typename... Args> inline void sd(const Args&... /*unused*/) {}
        template <typename Proto, typename Vector> inline auto se(Proto& proto, const Vector& vector) -> decltype(proto.se(), void()) { proto.set_se(vector[14]); }
        template <typename... Args> inline void se(const Args&... /*unused*/) {}
        template <typename Proto, typename Vector> inline auto sf(Proto& proto, const Vector& vector) -> decltype(proto.sf(), void()) { proto.set_sf(vector[15]); }
        template <typename... Args> inline void sf(const Args&... /*unused*/) {}
        // clang-format on
    }  // namespace set_protobuf_from_vector

    /**
     * @brief SFINAE functions to set vector values from protocol buffers
     */
    namespace set_vector_from_protobuf {
        // clang-format off
        template <typename Proto, typename Vector> inline auto x(Vector&& vector, const Proto& proto) -> decltype(proto.x(), void()) { vector[0] = proto.x(); }
        template <typename... Args> inline void x(const Args&... /*unused*/) {}
        template <typename Proto, typename Vector> inline auto y(Vector&& vector, const Proto& proto) -> decltype(proto.y(), void()) { vector[1] = proto.y(); }
        template <typename... Args> inline void y(const Args&... /*unused*/) {}
        template <typename Proto, typename Vector> inline auto z(Vector&& vector, const Proto& proto) -> decltype(proto.z(), void()) { vector[2] = proto.z(); }
        template <typename... Args> inline void z(const Args&... /*unused*/) {}
        template <typename Proto, typename Vector> inline auto t(Vector&& vector, const Proto& proto) -> decltype(proto.t(), void()) { vector[3] = proto.t(); }
        template <typename... Args> inline void t(const Args&... /*unused*/) {}

        template <typename Proto, typename Vector> inline auto s0(Vector&& vector, const Proto& proto) -> decltype(proto.s0(), void()) { vector[0] = proto.s0(); }
        template <typename... Args> inline void s0(const Args&... /*unused*/) {}
        template <typename Proto, typename Vector> inline auto s1(Vector&& vector, const Proto& proto) -> decltype(proto.s1(), void()) { vector[1] = proto.s1(); }
        template <typename... Args> inline void s1(const Args&... /*unused*/) {}
        template <typename Proto, typename Vector> inline auto s2(Vector&& vector, const Proto& proto) -> decltype(proto.s2(), void()) { vector[2] = proto.s2(); }
        template <typename... Args> inline void s2(const Args&... /*unused*/) {}
        template <typename Proto, typename Vector> inline auto s3(Vector&& vector, const Proto& proto) -> decltype(proto.s3(), void()) { vector[3] = proto.s3(); }
        template <typename... Args> inline void s3(const Args&... /*unused*/) {}
        template <typename Proto, typename Vector> inline auto s4(Vector&& vector, const Proto& proto) -> decltype(proto.s4(), void()) { vector[4] = proto.s4(); }
        template <typename... Args> inline void s4(const Args&... /*unused*/) {}
        template <typename Proto, typename Vector> inline auto s5(Vector&& vector, const Proto& proto) -> decltype(proto.s5(), void()) { vector[5] = proto.s5(); }
        template <typename... Args> inline void s5(const Args&... /*unused*/) {}
        template <typename Proto, typename Vector> inline auto s6(Vector&& vector, const Proto& proto) -> decltype(proto.s6(), void()) { vector[6] = proto.s6(); }
        template <typename... Args> inline void s6(const Args&... /*unused*/) {}
        template <typename Proto, typename Vector> inline auto s7(Vector&& vector, const Proto& proto) -> decltype(proto.s7(), void()) { vector[7] = proto.s7(); }
        template <typename... Args> inline void s7(const Args&... /*unused*/) {}
        template <typename Proto, typename Vector> inline auto s8(Vector&& vector, const Proto& proto) -> decltype(proto.s8(), void()) { vector[8] = proto.s8(); }
        template <typename... Args> inline void s8(const Args&... /*unused*/) {}
        template <typename Proto, typename Vector> inline auto s9(Vector&& vector, const Proto& proto) -> decltype(proto.s9(), void()) { vector[9] = proto.s9(); }
        template <typename... Args> inline void s9(const Args&... /*unused*/) {}
        template <typename Proto, typename Vector> inline auto sa(Vector&& vector, const Proto& proto) -> decltype(proto.sa(), void()) { vector[10] = proto.sa(); }
        template <typename... Args> inline void sa(const Args&... /*unused*/) {}
        template <typename Proto, typename Vector> inline auto sb(Vector&& vector, const Proto& proto) -> decltype(proto.sb(), void()) { vector[11] = proto.sb(); }
        template <typename... Args> inline void sb(const Args&... /*unused*/) {}
        template <typename Proto, typename Vector> inline auto sc(Vector&& vector, const Proto& proto) -> decltype(proto.sc(), void()) { vector[12] = proto.sc(); }
        template <typename... Args> inline void sc(const Args&... /*unused*/) {}
        template <typename Proto, typename Vector> inline auto sd(Vector&& vector, const Proto& proto) -> decltype(proto.sd(), void()) { vector[13] = proto.sd(); }
        template <typename... Args> inline void sd(const Args&... /*unused*/) {}
        template <typename Proto, typename Vector> inline auto se(Vector&& vector, const Proto& proto) -> decltype(proto.se(), void()) { vector[14] = proto.se(); }
        template <typename... Args> inline void se(const Args&... /*unused*/) {}
        template <typename Proto, typename Vector> inline auto sf(Vector&& vector, const Proto& proto) -> decltype(proto.sf(), void()) { vector[15] = proto.sf(); }
        template <typename... Args> inline void sf(const Args&... /*unused*/) {}
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
        inline void x(const Args&... /*unused*/) {}

        template <typename Proto, typename Matrix>
        inline auto y(Proto& proto, const Matrix& matrix) -> decltype(proto.y(), void()) {
            set_protobuf_from_vector::x(*proto.mutable_y(), matrix.col(1));
            set_protobuf_from_vector::y(*proto.mutable_y(), matrix.col(1));
            set_protobuf_from_vector::z(*proto.mutable_y(), matrix.col(1));
            set_protobuf_from_vector::t(*proto.mutable_y(), matrix.col(1));
        }
        template <typename... Args>
        inline void y(const Args&... /*unused*/) {}

        template <typename Proto, typename Matrix>
        inline auto z(Proto& proto, const Matrix& matrix) -> decltype(proto.z(), void()) {
            set_protobuf_from_vector::x(*proto.mutable_z(), matrix.col(2));
            set_protobuf_from_vector::y(*proto.mutable_z(), matrix.col(2));
            set_protobuf_from_vector::z(*proto.mutable_z(), matrix.col(2));
            set_protobuf_from_vector::t(*proto.mutable_z(), matrix.col(2));
        }
        template <typename... Args>
        inline void z(const Args&... /*unused*/) {}

        template <typename Proto, typename Matrix>
        inline auto t(Proto& proto, const Matrix& matrix) -> decltype(proto.t(), void()) {
            set_protobuf_from_vector::x(*proto.mutable_t(), matrix.col(3));
            set_protobuf_from_vector::y(*proto.mutable_t(), matrix.col(3));
            set_protobuf_from_vector::z(*proto.mutable_t(), matrix.col(3));
            set_protobuf_from_vector::t(*proto.mutable_t(), matrix.col(3));
        }
        template <typename... Args>
        inline void t(const Args&... /*unused*/) {}

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
        inline void s0(const Args&... /*unused*/) {}

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
        inline void s1(const Args&... /*unused*/) {}

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
        inline void s2(const Args&... /*unused*/) {}

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
        inline void s3(const Args&... /*unused*/) {}

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
        inline void s4(const Args&... /*unused*/) {}

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
        inline void s5(const Args&... /*unused*/) {}

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
        inline void s6(const Args&... /*unused*/) {}

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
        inline void s7(const Args&... /*unused*/) {}

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
        inline void s8(const Args&... /*unused*/) {}

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
        inline void s9(const Args&... /*unused*/) {}

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
        inline void sa(const Args&... /*unused*/) {}

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
        inline void sb(const Args&... /*unused*/) {}

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
        inline void sc(const Args&... /*unused*/) {}

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
        inline void sd(const Args&... /*unused*/) {}

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
        inline void se(const Args&... /*unused*/) {}

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
        inline void sf(const Args&... /*unused*/) {}

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
        inline void x(Args&... /*unused*/) {}

        template <typename Proto, typename Matrix>
        inline auto y(Matrix& matrix, const Proto& proto) -> decltype(proto.y(), void()) {
            set_vector_from_protobuf::x(matrix.col(1), proto.y());
            set_vector_from_protobuf::y(matrix.col(1), proto.y());
            set_vector_from_protobuf::z(matrix.col(1), proto.y());
            set_vector_from_protobuf::t(matrix.col(1), proto.y());
        }
        template <typename... Args>
        inline void y(Args&... /*unused*/) {}

        template <typename Proto, typename Matrix>
        inline auto z(Matrix& matrix, const Proto& proto) -> decltype(proto.z(), void()) {
            set_vector_from_protobuf::x(matrix.col(2), proto.z());
            set_vector_from_protobuf::y(matrix.col(2), proto.z());
            set_vector_from_protobuf::z(matrix.col(2), proto.z());
            set_vector_from_protobuf::t(matrix.col(2), proto.z());
        }
        template <typename... Args>
        inline void z(Args&... /*unused*/) {}

        template <typename Proto, typename Matrix>
        inline auto t(Matrix& matrix, const Proto& proto) -> decltype(proto.t(), void()) {
            set_vector_from_protobuf::x(matrix.col(3), proto.t());
            set_vector_from_protobuf::y(matrix.col(3), proto.t());
            set_vector_from_protobuf::z(matrix.col(3), proto.t());
            set_vector_from_protobuf::t(matrix.col(3), proto.t());
        }
        template <typename... Args>
        inline void t(Args&... /*unused*/) {}

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
        inline void s0(const Args&... /*unused*/) {}

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
        inline void s1(const Args&... /*unused*/) {}

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
        inline void s2(const Args&... /*unused*/) {}

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
        inline void s3(const Args&... /*unused*/) {}

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
        inline void s4(const Args&... /*unused*/) {}

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
        inline void s5(const Args&... /*unused*/) {}

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
        inline void s6(const Args&... /*unused*/) {}

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
        inline void s7(const Args&... /*unused*/) {}

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
        inline void s8(const Args&... /*unused*/) {}

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
        inline void s9(const Args&... /*unused*/) {}

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
        inline void sa(const Args&... /*unused*/) {}

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
        inline void sb(const Args&... /*unused*/) {}

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
        inline void sc(const Args&... /*unused*/) {}

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
        inline void sd(const Args&... /*unused*/) {}

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
        inline void se(const Args&... /*unused*/) {}

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
        inline void sf(const Args&... /*unused*/) {}
    }  // namespace set_matrix_from_protobuf

}  // namespace message::conversion

#endif  // MESSAGE_CONVERSION_PROTO_NEUTRON_SFINAE_HPP
