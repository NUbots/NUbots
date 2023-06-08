
#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <iomanip>
#include <numeric>
#include <string>
#include <vector>

#include "neutron/MessageTest.hpp"
#include "protobuf/MessageTest.pb.h"

using PbMessageTest = protobuf::message::nuclear::tests::MessageTest;
using message::nuclear::tests::MessageTest;

template <typename Scalar, int R, int C>
void set(Eigen::Matrix<Scalar, R, C>& A) {
    auto value = Scalar(1);
    for (int c = 0; c < C; ++c) {
        for (int r = 0; r < R; ++r) {
            A(r, c) = value;
            value += Scalar(1);
        }
    }
}

template <typename T>
bool check_vec(const T& a, const T& b) {
    std::vector<int> idx(a.v_size(), 0);
    std::iota(idx.begin(), idx.end(), 0);
    return a.v_size() == b.v_size()
           && std::all_of(idx.begin(), idx.end(), [&](const int& i) { return a.v(i) == b.v(i); });
}
template <>
bool check_vec<::cvec>(const ::cvec& a, const ::cvec& b) {
    return a.v() == b.v();
}
template <typename T>
bool check_mat(const T& a, const T& b) {
    std::vector<int> idx(a.v_size(), 0);
    std::iota(idx.begin(), idx.end(), 0);
    return a.v_size() == b.v_size() && a.rows() == b.rows() && a.cols() == b.cols()
           && std::all_of(idx.begin(), idx.end(), [&](const int& i) { return a.v(i) == b.v(i); });
}
template <>
bool check_mat<::cmat>(const ::cmat& a, const ::cmat& b) {
    return a.rows() == b.rows() && a.cols() == b.cols() && a.v() == b.v();
}
// clang-format off
template<typename T>
bool check_vec2(const T& a, const T& b) {
    return a.x() == b.x() && a.y() == b.y();
}
template<typename T>
bool check_vec3(const T& a, const T& b) {
    return a.x() == b.x() && a.y() == b.y() && a.z() == b.z();
}
template<typename T>
bool check_vec4(const T& a, const T& b) {
    return a.x() == b.x() && a.y() == b.y() && a.z() == b.z() && a.t() == b.t();
}
template<typename T>
bool check_quat(const T& a, const T& b) {
    return a.x() == b.x() && a.y() == b.y() && a.z() == b.z() && a.w() == b.w();
}
template<typename T>
bool check_vec5(const T& a, const T& b) {
    return a.s0() == b.s0() && a.s1() == b.s1() && a.s2() == b.s2() && a.s3() == b.s3() && a.s4() == b.s4();
}
template<typename T>
bool check_vec6(const T& a, const T& b) {
    return a.s0() == b.s0() && a.s1() == b.s1() && a.s2() == b.s2() && a.s3() == b.s3() && a.s4() == b.s4() &&
           a.s5() == b.s5();
}
template<typename T>
bool check_vec7(const T& a, const T& b) {
    return a.s0() == b.s0() && a.s1() == b.s1() && a.s2() == b.s2() && a.s3() == b.s3() && a.s4() == b.s4() &&
           a.s5() == b.s5() && a.s6() == b.s6();
}
template<typename T>
bool check_vec8(const T& a, const T& b) {
    return a.s0() == b.s0() && a.s1() == b.s1() && a.s2() == b.s2() && a.s3() == b.s3() && a.s4() == b.s4() &&
           a.s5() == b.s5() && a.s6() == b.s6() && a.s7() == b.s7();
}
template<typename T>
bool check_vec9(const T& a, const T& b) {
    return a.s0() == b.s0() && a.s1() == b.s1() && a.s2() == b.s2() && a.s3() == b.s3() && a.s4() == b.s4() &&
           a.s5() == b.s5() && a.s6() == b.s6() && a.s7() == b.s7() && a.s8() == b.s8();
}
template<typename T>
bool check_vec10(const T& a, const T& b) {
    return a.s0() == b.s0() && a.s1() == b.s1() && a.s2() == b.s2() && a.s3() == b.s3() && a.s4() == b.s4() &&
           a.s5() == b.s5() && a.s6() == b.s6() && a.s7() == b.s7() && a.s8() == b.s8() && a.s9() == b.s9();
}
template<typename T>
bool check_vec11(const T& a, const T& b) {
    return a.s0() == b.s0() && a.s1() == b.s1() && a.s2() == b.s2() && a.s3() == b.s3() && a.s4() == b.s4() &&
           a.s5() == b.s5() && a.s6() == b.s6() && a.s7() == b.s7() && a.s8() == b.s8() && a.s9() == b.s9() &&
           a.sa() == b.sa();
}
template<typename T>
bool check_vec12(const T& a, const T& b) {
    return a.s0() == b.s0() && a.s1() == b.s1() && a.s2() == b.s2() && a.s3() == b.s3() && a.s4() == b.s4() &&
           a.s5() == b.s5() && a.s6() == b.s6() && a.s7() == b.s7() && a.s8() == b.s8() && a.s9() == b.s9() &&
           a.sa() == b.sa() && a.sb() == b.sb();
}
template<typename T>
bool check_vec13(const T& a, const T& b) {
    return a.s0() == b.s0() && a.s1() == b.s1() && a.s2() == b.s2() && a.s3() == b.s3() && a.s4() == b.s4() &&
           a.s5() == b.s5() && a.s6() == b.s6() && a.s7() == b.s7() && a.s8() == b.s8() && a.s9() == b.s9() &&
           a.sa() == b.sa() && a.sb() == b.sb() && a.sc() == b.sc();
}
template<typename T>
bool check_vec14(const T& a, const T& b) {
    return a.s0() == b.s0() && a.s1() == b.s1() && a.s2() == b.s2() && a.s3() == b.s3() && a.s4() == b.s4() &&
           a.s5() == b.s5() && a.s6() == b.s6() && a.s7() == b.s7() && a.s8() == b.s8() && a.s9() == b.s9() &&
           a.sa() == b.sa() && a.sb() == b.sb() && a.sc() == b.sc() && a.sd() == b.sd();
}
template<typename T>
bool check_vec15(const T& a, const T& b) {
    return a.s0() == b.s0() && a.s1() == b.s1() && a.s2() == b.s2() && a.s3() == b.s3() && a.s4() == b.s4() &&
           a.s5() == b.s5() && a.s6() == b.s6() && a.s7() == b.s7() && a.s8() == b.s8() && a.s9() == b.s9() &&
           a.sa() == b.sa() && a.sb() == b.sb() && a.sc() == b.sc() && a.sd() == b.sd() && a.se() == b.se();
}
template<typename T>
bool check_vec16(const T& a, const T& b) {
    return a.s0() == b.s0() && a.s1() == b.s1() && a.s2() == b.s2() && a.s3() == b.s3() && a.s4() == b.s4() &&
           a.s5() == b.s5() && a.s6() == b.s6() && a.s7() == b.s7() && a.s8() == b.s8() && a.s9() == b.s9() &&
           a.sa() == b.sa() && a.sb() == b.sb() && a.sc() == b.sc() && a.sd() == b.sd() && a.se() == b.se() &&
           a.sf() == b.sf();
}
template<typename T>
bool check_mat2(const T& a, const T& b) {
    return check_vec2(a.x(), b.x()) && check_vec2(a.y(), b.y());
}
template<typename T>
bool check_mat3(const T& a, const T& b) {
    return check_vec3(a.x(), b.x()) && check_vec3(a.y(), b.y()) && check_vec3(a.z(), b.z());
}
template<typename T>
bool check_mat4(const T& a, const T& b) {
    return check_vec4(a.x(), b.x()) && check_vec4(a.y(), b.y()) && check_vec4(a.z(), b.z()) && check_vec4(a.t(), b.t());
}
template<typename T>
bool check_mat5(const T& a, const T& b) {
    return check_vec5(a.s0(), b.s0()) && check_vec5(a.s1(), b.s1()) && check_vec5(a.s2(), b.s2()) &&
           check_vec5(a.s3(), b.s3()) && check_vec5(a.s4(), b.s4());
}
template<typename T>
bool check_mat6(const T& a, const T& b) {
    return check_vec6(a.s0(), b.s0()) && check_vec6(a.s1(), b.s1()) && check_vec6(a.s2(), b.s2()) &&
           check_vec6(a.s3(), b.s3()) && check_vec6(a.s4(), b.s4()) && check_vec6(a.s5(), b.s5());
}
template<typename T>
bool check_mat7(const T& a, const T& b) {
    return check_vec7(a.s0(), b.s0()) && check_vec7(a.s1(), b.s1()) && check_vec7(a.s2(), b.s2()) &&
           check_vec7(a.s3(), b.s3()) && check_vec7(a.s4(), b.s4()) && check_vec7(a.s5(), b.s5()) &&
           check_vec7(a.s6(), b.s6());
}
template<typename T>
bool check_mat8(const T& a, const T& b) {
    return check_vec8(a.s0(), b.s0()) && check_vec8(a.s1(), b.s1()) && check_vec8(a.s2(), b.s2()) &&
           check_vec8(a.s3(), b.s3()) && check_vec8(a.s4(), b.s4()) && check_vec8(a.s5(), b.s5()) &&
           check_vec8(a.s6(), b.s6()) && check_vec8(a.s7(), b.s7());
}
template<typename T>
bool check_mat9(const T& a, const T& b) {
    return check_vec9(a.s0(), b.s0()) && check_vec9(a.s1(), b.s1()) && check_vec9(a.s2(), b.s2()) &&
           check_vec9(a.s3(), b.s3()) && check_vec9(a.s4(), b.s4()) && check_vec9(a.s5(), b.s5()) &&
           check_vec9(a.s6(), b.s6()) && check_vec9(a.s7(), b.s7()) && check_vec9(a.s8(), b.s8());
}
template<typename T>
bool check_mat10(const T& a, const T& b) {
    return check_vec10(a.s0(), b.s0()) && check_vec10(a.s1(), b.s1()) && check_vec10(a.s2(), b.s2()) &&
           check_vec10(a.s3(), b.s3()) && check_vec10(a.s4(), b.s4()) && check_vec10(a.s5(), b.s5()) &&
           check_vec10(a.s6(), b.s6()) && check_vec10(a.s7(), b.s7()) && check_vec10(a.s8(), b.s8()) &&
           check_vec10(a.s9(), b.s9());
}
template<typename T>
bool check_mat11(const T& a, const T& b) {
    return check_vec11(a.s0(), b.s0()) && check_vec11(a.s1(), b.s1()) && check_vec11(a.s2(), b.s2()) &&
           check_vec11(a.s3(), b.s3()) && check_vec11(a.s4(), b.s4()) && check_vec11(a.s5(), b.s5()) &&
           check_vec11(a.s6(), b.s6()) && check_vec11(a.s7(), b.s7()) && check_vec11(a.s8(), b.s8()) &&
           check_vec11(a.s9(), b.s9()) && check_vec11(a.sa(), b.sa());
}
template<typename T>
bool check_mat12(const T& a, const T& b) {
    return check_vec12(a.s0(), b.s0()) && check_vec12(a.s1(), b.s1()) && check_vec12(a.s2(), b.s2()) &&
           check_vec12(a.s3(), b.s3()) && check_vec12(a.s4(), b.s4()) && check_vec12(a.s5(), b.s5()) &&
           check_vec12(a.s6(), b.s6()) && check_vec12(a.s7(), b.s7()) && check_vec12(a.s8(), b.s8()) &&
           check_vec12(a.s9(), b.s9()) && check_vec12(a.sa(), b.sa()) && check_vec12(a.sb(), b.sb());
}
template<typename T>
bool check_mat13(const T& a, const T& b) {
    return check_vec13(a.s0(), b.s0()) && check_vec13(a.s1(), b.s1()) && check_vec13(a.s2(), b.s2()) &&
           check_vec13(a.s3(), b.s3()) && check_vec13(a.s4(), b.s4()) && check_vec13(a.s5(), b.s5()) &&
           check_vec13(a.s6(), b.s6()) && check_vec13(a.s7(), b.s7()) && check_vec13(a.s8(), b.s8()) &&
           check_vec13(a.s9(), b.s9()) && check_vec13(a.sa(), b.sa()) && check_vec13(a.sb(), b.sb()) &&
           check_vec13(a.sc(), b.sc());
}
template<typename T>
bool check_mat14(const T& a, const T& b) {
    return check_vec14(a.s0(), b.s0()) && check_vec14(a.s1(), b.s1()) && check_vec14(a.s2(), b.s2()) &&
           check_vec14(a.s3(), b.s3()) && check_vec14(a.s4(), b.s4()) && check_vec14(a.s5(), b.s5()) &&
           check_vec14(a.s6(), b.s6()) && check_vec14(a.s7(), b.s7()) && check_vec14(a.s8(), b.s8()) &&
           check_vec14(a.s9(), b.s9()) && check_vec14(a.sa(), b.sa()) && check_vec14(a.sb(), b.sb()) &&
           check_vec14(a.sc(), b.sc()) && check_vec14(a.sd(), b.sd());
}
template<typename T>
bool check_mat15(const T& a, const T& b) {
    return check_vec15(a.s0(), b.s0()) && check_vec15(a.s1(), b.s1()) && check_vec15(a.s2(), b.s2()) &&
           check_vec15(a.s3(), b.s3()) && check_vec15(a.s4(), b.s4()) && check_vec15(a.s5(), b.s5()) &&
           check_vec15(a.s6(), b.s6()) && check_vec15(a.s7(), b.s7()) && check_vec15(a.s8(), b.s8()) &&
           check_vec15(a.s9(), b.s9()) && check_vec15(a.sa(), b.sa()) && check_vec15(a.sb(), b.sb()) &&
           check_vec15(a.sc(), b.sc()) && check_vec15(a.sd(), b.sd()) && check_vec15(a.se(), b.se());
}
template<typename T>
bool check_mat16(const T& a, const T& b) {
    return check_vec16(a.s0(), b.s0()) && check_vec16(a.s1(), b.s1()) && check_vec16(a.s2(), b.s2()) &&
           check_vec16(a.s3(), b.s3()) && check_vec16(a.s4(), b.s4()) && check_vec16(a.s5(), b.s5()) &&
           check_vec16(a.s6(), b.s6()) && check_vec16(a.s7(), b.s7()) && check_vec16(a.s8(), b.s8()) &&
           check_vec16(a.s9(), b.s9()) && check_vec16(a.sa(), b.sa()) && check_vec16(a.sb(), b.sb()) &&
           check_vec16(a.sc(), b.sc()) && check_vec16(a.sd(), b.sd()) && check_vec16(a.se(), b.se()) &&
           check_vec16(a.sf(), b.sf()); }
// clang-format on

// clang-format off
template<typename T, typename U>
void set_vec2(T* vec, const U& N) {
    vec->set_x(N+U(0)); vec->set_y(N+U(1));
}
template<typename T, typename U>
void set_vec3(T* vec, const U& N) {
    vec->set_x(N+U{0}); vec->set_y(N+U(1)); vec->set_z(N+U(2));
}
template<typename T, typename U>
void set_vec4(T* vec, const U& N) {
    vec->set_x(N+U(0)); vec->set_y(N+U(1)); vec->set_z(N+U(2)); vec->set_t(N+U(3));
}
template<typename T, typename U>
void set_quat(T* vec, const U& N) {
    vec->set_x(N+U(0)); vec->set_y(N+U(1)); vec->set_z(N+U(2)); vec->set_w(N+U(3));
}
template<typename T, typename U>
void set_vec5(T* vec, const U& N) {
    vec->set_s0(N+U(0)); vec->set_s1(N+U(1)); vec->set_s2(N+U(2)); vec->set_s3(N+U(3)); vec->set_s4(N+U(4));
}
template<typename T, typename U>
void set_vec6(T* vec, const U& N) {
    vec->set_s0(N+U(0)); vec->set_s1(N+U(1)); vec->set_s2(N+U(2)); vec->set_s3(N+U(3)); vec->set_s4(N+U(4));
    vec->set_s5(N+U(5));
}
template<typename T, typename U>
void set_vec7(T* vec, const U& N) {
    vec->set_s0(N+U(0)); vec->set_s1(N+U(1)); vec->set_s2(N+U(2)); vec->set_s3(N+U(3)); vec->set_s4(N+U(4));
    vec->set_s5(N+U(5)); vec->set_s6(N+U(6));
}
template<typename T, typename U>
void set_vec8(T* vec, const U& N) {
    vec->set_s0(N+U(0)); vec->set_s1(N+U(1)); vec->set_s2(N+U(2)); vec->set_s3(N+U(3)); vec->set_s4(N+U(4));
    vec->set_s5(N+U(5)); vec->set_s6(N+U(6)); vec->set_s7(N+U(7));
}
template<typename T, typename U>
void set_vec9(T* vec, const U& N) {
    vec->set_s0(N+U(0)); vec->set_s1(N+U(1)); vec->set_s2(N+U(2)); vec->set_s3(N+U(3)); vec->set_s4(N+U(4));
    vec->set_s5(N+U(5)); vec->set_s6(N+U(6)); vec->set_s7(N+U(7)); vec->set_s8(N+U(8));
}
template<typename T, typename U>
void set_vec10(T* vec, const U& N) {
    vec->set_s0(N+U(0)); vec->set_s1(N+U(1)); vec->set_s2(N+U(2)); vec->set_s3(N+U(3)); vec->set_s4(N+U(4));
    vec->set_s5(N+U(5)); vec->set_s6(N+U(6)); vec->set_s7(N+U(7)); vec->set_s8(N+U(8)); vec->set_s9(N+U(9));
}
template<typename T, typename U>
void set_vec11(T* vec, const U& N) {
    vec->set_s0(N+U(0)); vec->set_s1(N+U(1)); vec->set_s2(N+U(2)); vec->set_s3(N+U(3)); vec->set_s4(N+U(4));
    vec->set_s5(N+U(5)); vec->set_s6(N+U(6)); vec->set_s7(N+U(7)); vec->set_s8(N+U(8)); vec->set_s9(N+U(9));
    vec->set_sa(N+U(10));
}
template<typename T, typename U>
void set_vec12(T* vec, const U& N) {
    vec->set_s0(N+U(0)); vec->set_s1(N+U(1)); vec->set_s2(N+U(2)); vec->set_s3(N+U(3)); vec->set_s4(N+U(4));
    vec->set_s5(N+U(5)); vec->set_s6(N+U(6)); vec->set_s7(N+U(7)); vec->set_s8(N+U(8)); vec->set_s9(N+U(9));
    vec->set_sa(N+U(10)); vec->set_sb(N+U(11));
}
template<typename T, typename U>
void set_vec13(T* vec, const U& N) {
    vec->set_s0(N+U(0)); vec->set_s1(N+U(1)); vec->set_s2(N+U(2)); vec->set_s3(N+U(3)); vec->set_s4(N+U(4));
    vec->set_s5(N+U(5)); vec->set_s6(N+U(6)); vec->set_s7(N+U(7)); vec->set_s8(N+U(8)); vec->set_s9(N+U(9));
    vec->set_sa(N+U(10)); vec->set_sb(N+U(11)); vec->set_sc(N+U(12));
}
template<typename T, typename U>
void set_vec14(T* vec, const U& N) {
    vec->set_s0(N+U(0)); vec->set_s1(N+U(1)); vec->set_s2(N+U(2)); vec->set_s3(N+U(3)); vec->set_s4(N+U(4));
    vec->set_s5(N+U(5)); vec->set_s6(N+U(6)); vec->set_s7(N+U(7)); vec->set_s8(N+U(8)); vec->set_s9(N+U(9));
    vec->set_sa(N+U(10)); vec->set_sb(N+U(11)); vec->set_sc(N+U(12)); vec->set_sd(N+U(13));
}
template<typename T, typename U>
void set_vec15(T* vec, const U& N) {
    vec->set_s0(N+U(0)); vec->set_s1(N+U(1)); vec->set_s2(N+U(2)); vec->set_s3(N+U(3)); vec->set_s4(N+U(4));
    vec->set_s5(N+U(5)); vec->set_s6(N+U(6)); vec->set_s7(N+U(7)); vec->set_s8(N+U(8)); vec->set_s9(N+U(9));
    vec->set_sa(N+U(10)); vec->set_sb(N+U(11)); vec->set_sc(N+U(12)); vec->set_sd(N+U(13)); vec->set_se(N+U(14));
}
template<typename T, typename U>
void set_vec16(T* vec, const U& N) {
    vec->set_s0(N+U(0)); vec->set_s1(N+U(1)); vec->set_s2(N+U(2)); vec->set_s3(N+U(3)); vec->set_s4(N+U(4));
    vec->set_s5(N+U(5)); vec->set_s6(N+U(6)); vec->set_s7(N+U(7)); vec->set_s8(N+U(8)); vec->set_s9(N+U(9));
    vec->set_sa(N+U(10)); vec->set_sb(N+U(11)); vec->set_sc(N+U(12)); vec->set_sd(N+U(13)); vec->set_se(N+U(14));
    vec->set_sf(N+U(15));
}

template<typename T, typename U>
void set_mat2(T* mat, const U& N) {
    set_vec2(mat->mutable_x(), N+U(0*2)); set_vec2(mat->mutable_y(), N+U(1*2));
}
template<typename T, typename U>
void set_mat3(T* mat, const U& N) {
    set_vec3(mat->mutable_x(), N+U(0*3)); set_vec3(mat->mutable_y(), N+U(1*3)); set_vec3(mat->mutable_z(), N+U(2*3));
}
template<typename T, typename U>
void set_mat4(T* mat, const U& N) {
    set_vec4(mat->mutable_x(), N+U(0*4)); set_vec4(mat->mutable_y(), N+U(1*4)); set_vec4(mat->mutable_z(), N+U(2*4));
    set_vec4(mat->mutable_t(), N+U(3*4));
}
template<typename T, typename U>
void set_mat5(T* mat, const U& N) {
    set_vec5(mat->mutable_s0(), N+U(0*5)); set_vec5(mat->mutable_s1(), N+U(1*5)); set_vec5(mat->mutable_s2(), N+U(2*5));
    set_vec5(mat->mutable_s3(), N+U(3*5)); set_vec5(mat->mutable_s4(), N+U(4*5));
}
template<typename T, typename U>
void set_mat6(T* mat, const U& N) {
    set_vec6(mat->mutable_s0(), N+U(0*6)); set_vec6(mat->mutable_s1(), N+U(1*6)); set_vec6(mat->mutable_s2(), N+U(2*6));
    set_vec6(mat->mutable_s3(), N+U(3*6)); set_vec6(mat->mutable_s4(), N+U(4*6)); set_vec6(mat->mutable_s5(), N+U(5*6));
}
template<typename T, typename U>
void set_mat7(T* mat, const U& N) {
    set_vec7(mat->mutable_s0(), N+U(0*7)); set_vec7(mat->mutable_s1(), N+U(1*7)); set_vec7(mat->mutable_s2(), N+U(2*7));
    set_vec7(mat->mutable_s3(), N+U(3*7)); set_vec7(mat->mutable_s4(), N+U(4*7)); set_vec7(mat->mutable_s5(), N+U(5*7));
    set_vec7(mat->mutable_s6(), N+U(6*7));
}
template<typename T, typename U>
void set_mat8(T* mat, const U& N) {
    set_vec8(mat->mutable_s0(), N+U(0*8)); set_vec8(mat->mutable_s1(), N+U(1*8)); set_vec8(mat->mutable_s2(), N+U(2*8));
    set_vec8(mat->mutable_s3(), N+U(3*8)); set_vec8(mat->mutable_s4(), N+U(4*8)); set_vec8(mat->mutable_s5(), N+U(5*8));
    set_vec8(mat->mutable_s6(), N+U(6*8)); set_vec8(mat->mutable_s7(), N+U(7*8));
}
template<typename T, typename U>
void set_mat9(T* mat, const U& N) {
    set_vec9(mat->mutable_s0(), N+U(0*9)); set_vec9(mat->mutable_s1(), N+U(1*9)); set_vec9(mat->mutable_s2(), N+U(2*9));
    set_vec9(mat->mutable_s3(), N+U(3*9)); set_vec9(mat->mutable_s4(), N+U(4*9)); set_vec9(mat->mutable_s5(), N+U(5*9));
    set_vec9(mat->mutable_s6(), N+U(6*9)); set_vec9(mat->mutable_s7(), N+U(7*9)); set_vec9(mat->mutable_s8(), N+U(8*9));
}
template<typename T, typename U>
void set_mat10(T* mat, const U& N) {
    set_vec10(mat->mutable_s0(), N+U(0*10)); set_vec10(mat->mutable_s1(), N+U(1*10));
    set_vec10(mat->mutable_s2(), N+U(2*10)); set_vec10(mat->mutable_s3(), N+U(3*10));
    set_vec10(mat->mutable_s4(), N+U(4*10)); set_vec10(mat->mutable_s5(), N+U(5*10));
    set_vec10(mat->mutable_s6(), N+U(6*10)); set_vec10(mat->mutable_s7(), N+U(7*10));
    set_vec10(mat->mutable_s8(), N+U(8*10)); set_vec10(mat->mutable_s9(), N+U(9*10));
}
template<typename T, typename U>
void set_mat11(T* mat, const U& N) {
    set_vec11(mat->mutable_s0(), N+U(0*11)); set_vec11(mat->mutable_s1(), N+U(1*11));
    set_vec11(mat->mutable_s2(), N+U(2*11)); set_vec11(mat->mutable_s3(), N+U(3*11));
    set_vec11(mat->mutable_s4(), N+U(4*11)); set_vec11(mat->mutable_s5(), N+U(5*11));
    set_vec11(mat->mutable_s6(), N+U(6*11)); set_vec11(mat->mutable_s7(), N+U(7*11));
    set_vec11(mat->mutable_s8(), N+U(8*11)); set_vec11(mat->mutable_s9(), N+U(9*11));
    set_vec11(mat->mutable_sa(), N+U(10*11));
}
template<typename T, typename U>
void set_mat12(T* mat, const U& N) {
    set_vec12(mat->mutable_s0(), N+U(0*12)); set_vec12(mat->mutable_s1(), N+U(1*12));
    set_vec12(mat->mutable_s2(), N+U(2*12)); set_vec12(mat->mutable_s3(), N+U(3*12));
    set_vec12(mat->mutable_s4(), N+U(4*12)); set_vec12(mat->mutable_s5(), N+U(5*12));
    set_vec12(mat->mutable_s6(), N+U(6*12)); set_vec12(mat->mutable_s7(), N+U(7*12));
    set_vec12(mat->mutable_s8(), N+U(8*12)); set_vec12(mat->mutable_s9(), N+U(9*12));
    set_vec12(mat->mutable_sa(), N+U(10*12)); set_vec12(mat->mutable_sb(), N+U(11*12));
}
template<typename T, typename U>
void set_mat13(T* mat, const U& N) {
    set_vec13(mat->mutable_s0(), N+U(0*13)); set_vec13(mat->mutable_s1(), N+U(1*13));
    set_vec13(mat->mutable_s2(), N+U(2*13)); set_vec13(mat->mutable_s3(), N+U(3*13));
    set_vec13(mat->mutable_s4(), N+U(4*13)); set_vec13(mat->mutable_s5(), N+U(5*13));
    set_vec13(mat->mutable_s6(), N+U(6*13)); set_vec13(mat->mutable_s7(), N+U(7*13));
    set_vec13(mat->mutable_s8(), N+U(8*13)); set_vec13(mat->mutable_s9(), N+U(9*13));
    set_vec13(mat->mutable_sa(), N+U(10*13)); set_vec13(mat->mutable_sb(), N+U(11*13));
    set_vec13(mat->mutable_sc(), N+U(12*13));
}
template<typename T, typename U>
void set_mat14(T* mat, const U& N) {
    set_vec14(mat->mutable_s0(), N+U(0*14)); set_vec14(mat->mutable_s1(), N+U(1*14));
    set_vec14(mat->mutable_s2(), N+U(2*14)); set_vec14(mat->mutable_s3(), N+U(3*14));
    set_vec14(mat->mutable_s4(), N+U(4*14)); set_vec14(mat->mutable_s5(), N+U(5*14));
    set_vec14(mat->mutable_s6(), N+U(6*14)); set_vec14(mat->mutable_s7(), N+U(7*14));
    set_vec14(mat->mutable_s8(), N+U(8*14)); set_vec14(mat->mutable_s9(), N+U(9*14));
    set_vec14(mat->mutable_sa(), N+U(10*14)); set_vec14(mat->mutable_sb(), N+U(11*14));
    set_vec14(mat->mutable_sc(), N+U(12*14)); set_vec14(mat->mutable_sd(), N+U(13*14));
}
template<typename T, typename U>
void set_mat15(T* mat, const U& N) {
    set_vec15(mat->mutable_s0(), N+U(0*15)); set_vec15(mat->mutable_s1(), N+U(1*15));
    set_vec15(mat->mutable_s2(), N+U(2*15)); set_vec15(mat->mutable_s3(), N+U(3*15));
    set_vec15(mat->mutable_s4(), N+U(4*15)); set_vec15(mat->mutable_s5(), N+U(5*15));
    set_vec15(mat->mutable_s6(), N+U(6*15)); set_vec15(mat->mutable_s7(), N+U(7*15));
    set_vec15(mat->mutable_s8(), N+U(8*15)); set_vec15(mat->mutable_s9(), N+U(9*15));
    set_vec15(mat->mutable_sa(), N+U(10*15)); set_vec15(mat->mutable_sb(), N+U(11*15));
    set_vec15(mat->mutable_sc(), N+U(12*15)); set_vec15(mat->mutable_sd(), N+U(13*15));
    set_vec15(mat->mutable_se(), N+U(14*15));
}
template<typename T, typename U>
void set_mat16(T* mat, const U& N) {
    set_vec16(mat->mutable_s0(), N+U(0*16)); set_vec16(mat->mutable_s1(), N+U(1*16));
    set_vec16(mat->mutable_s2(), N+U(2*16)); set_vec16(mat->mutable_s3(), N+U(3*16));
    set_vec16(mat->mutable_s4(), N+U(4*16)); set_vec16(mat->mutable_s5(), N+U(5*16));
    set_vec16(mat->mutable_s6(), N+U(6*16)); set_vec16(mat->mutable_s7(), N+U(7*16));
    set_vec16(mat->mutable_s8(), N+U(8*16)); set_vec16(mat->mutable_s9(), N+U(9*16));
    set_vec16(mat->mutable_sa(), N+U(10*16)); set_vec16(mat->mutable_sb(), N+U(11*16));
    set_vec16(mat->mutable_sc(), N+U(12*16)); set_vec16(mat->mutable_sd(), N+U(13*16));
    set_vec16(mat->mutable_se(), N+U(14*16)); set_vec16(mat->mutable_sf(), N+U(15*16));
}
// clang-format on

PbMessageTest construct_pb_message(const NUClear::clock::time_point& tp) {
    PbMessageTest msg;

    // Compute seconds and nanoseconds for the timestamp and duration
    auto d       = tp.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(d);
    auto nanos   = std::chrono::duration_cast<std::chrono::nanoseconds>(d - seconds);

    // Populate timestamp and duration on message
    msg.mutable_timestamp()->set_seconds(seconds.count());
    msg.mutable_timestamp()->set_nanos(int32_t(nanos.count()));
    msg.mutable_duration()->set_seconds(seconds.count());
    msg.mutable_duration()->set_nanos(int32_t(nanos.count()));

    // Populate scalar field values
    msg.set_d(3.0);
    msg.set_f(4.0);
    msg.set_i32(5);
    msg.set_i64(6);
    msg.set_u32(7);
    msg.set_u64(8);
    msg.set_s32(9);
    msg.set_s64(10);
    msg.set_fixed32(11);
    msg.set_fixed64(12);
    msg.set_sfixed32(13);
    msg.set_sfixed64(14);
    msg.set_b(true);
    msg.set_s("16");
    msg.set_bytes("17");

    // Populate dynamic vector fields
    for (int i = 0; i < 5; ++i) {
        msg.mutable_vec()->add_v(double(i + 1));
        msg.mutable_fvec()->add_v(float(i + 1));
        msg.mutable_ivec()->add_v(i + 1);
        msg.mutable_uvec()->add_v(i + 1);
        msg.mutable_cvec()->mutable_v()->push_back(char(i + 1));
    }

    // Populate dynamic matrix fields
    msg.mutable_mat()->set_rows(5);
    msg.mutable_mat()->set_cols(5);
    msg.mutable_fmat()->set_rows(5);
    msg.mutable_fmat()->set_cols(5);
    msg.mutable_imat()->set_rows(5);
    msg.mutable_imat()->set_cols(5);
    msg.mutable_umat()->set_rows(5);
    msg.mutable_umat()->set_cols(5);
    msg.mutable_cmat()->set_rows(5);
    msg.mutable_cmat()->set_cols(5);
    for (int i = 0; i < 25; ++i) {
        msg.mutable_mat()->add_v(double(i + 1));
        msg.mutable_fmat()->add_v(float(i + 1));
        msg.mutable_imat()->add_v(i + 1);
        msg.mutable_umat()->add_v(i + 1);
        msg.mutable_cmat()->mutable_v()->push_back(char(i + 1));
    }

    // Populate fixed sized vectors
    set_vec2(msg.mutable_vec2(), 1.0);
    set_vec3(msg.mutable_vec3(), 1.0);
    set_vec4(msg.mutable_vec4(), 1.0);
    set_vec5(msg.mutable_vec5(), 1.0);
    set_vec6(msg.mutable_vec6(), 1.0);
    set_vec7(msg.mutable_vec7(), 1.0);
    set_vec8(msg.mutable_vec8(), 1.0);
    set_vec9(msg.mutable_vec9(), 1.0);
    set_vec10(msg.mutable_vec10(), 1.0);
    set_vec11(msg.mutable_vec11(), 1.0);
    set_vec12(msg.mutable_vec12(), 1.0);
    set_vec13(msg.mutable_vec13(), 1.0);
    set_vec14(msg.mutable_vec14(), 1.0);
    set_vec15(msg.mutable_vec15(), 1.0);
    set_vec16(msg.mutable_vec16(), 1.0);

    set_vec2(msg.mutable_fvec2(), 1.0f);
    set_vec3(msg.mutable_fvec3(), 1.0f);
    set_vec4(msg.mutable_fvec4(), 1.0f);
    set_vec5(msg.mutable_fvec5(), 1.0f);
    set_vec6(msg.mutable_fvec6(), 1.0f);
    set_vec7(msg.mutable_fvec7(), 1.0f);
    set_vec8(msg.mutable_fvec8(), 1.0f);
    set_vec9(msg.mutable_fvec9(), 1.0f);
    set_vec10(msg.mutable_fvec10(), 1.0f);
    set_vec11(msg.mutable_fvec11(), 1.0f);
    set_vec12(msg.mutable_fvec12(), 1.0f);
    set_vec13(msg.mutable_fvec13(), 1.0f);
    set_vec14(msg.mutable_fvec14(), 1.0f);
    set_vec15(msg.mutable_fvec15(), 1.0f);
    set_vec16(msg.mutable_fvec16(), 1.0f);

    set_vec2(msg.mutable_ivec2(), 1);
    set_vec3(msg.mutable_ivec3(), 1);
    set_vec4(msg.mutable_ivec4(), 1);
    set_vec5(msg.mutable_ivec5(), 1);
    set_vec6(msg.mutable_ivec6(), 1);
    set_vec7(msg.mutable_ivec7(), 1);
    set_vec8(msg.mutable_ivec8(), 1);
    set_vec9(msg.mutable_ivec9(), 1);
    set_vec10(msg.mutable_ivec10(), 1);
    set_vec11(msg.mutable_ivec11(), 1);
    set_vec12(msg.mutable_ivec12(), 1);
    set_vec13(msg.mutable_ivec13(), 1);
    set_vec14(msg.mutable_ivec14(), 1);
    set_vec15(msg.mutable_ivec15(), 1);
    set_vec16(msg.mutable_ivec16(), 1);

    set_vec2(msg.mutable_uvec2(), 1u);
    set_vec3(msg.mutable_uvec3(), 1u);
    set_vec4(msg.mutable_uvec4(), 1u);
    set_vec5(msg.mutable_uvec5(), 1u);
    set_vec6(msg.mutable_uvec6(), 1u);
    set_vec7(msg.mutable_uvec7(), 1u);
    set_vec8(msg.mutable_uvec8(), 1u);
    set_vec9(msg.mutable_uvec9(), 1u);
    set_vec10(msg.mutable_uvec10(), 1u);
    set_vec11(msg.mutable_uvec11(), 1u);
    set_vec12(msg.mutable_uvec12(), 1u);
    set_vec13(msg.mutable_uvec13(), 1u);
    set_vec14(msg.mutable_uvec14(), 1u);
    set_vec15(msg.mutable_uvec15(), 1u);
    set_vec16(msg.mutable_uvec16(), 1u);

    // Populate fixed sized matrices
    set_mat2(msg.mutable_mat2(), 1.0);
    set_mat3(msg.mutable_mat3(), 1.0);
    set_mat4(msg.mutable_mat4(), 1.0);
    set_mat5(msg.mutable_mat5(), 1.0);
    set_mat6(msg.mutable_mat6(), 1.0);
    set_mat7(msg.mutable_mat7(), 1.0);
    set_mat8(msg.mutable_mat8(), 1.0);
    set_mat9(msg.mutable_mat9(), 1.0);
    set_mat10(msg.mutable_mat10(), 1.0);
    set_mat11(msg.mutable_mat11(), 1.0);
    set_mat12(msg.mutable_mat12(), 1.0);
    set_mat13(msg.mutable_mat13(), 1.0);
    set_mat14(msg.mutable_mat14(), 1.0);
    set_mat15(msg.mutable_mat15(), 1.0);
    set_mat16(msg.mutable_mat16(), 1.0);

    set_mat2(msg.mutable_fmat2(), 1.0f);
    set_mat3(msg.mutable_fmat3(), 1.0f);
    set_mat4(msg.mutable_fmat4(), 1.0f);
    set_mat5(msg.mutable_fmat5(), 1.0f);
    set_mat6(msg.mutable_fmat6(), 1.0f);
    set_mat7(msg.mutable_fmat7(), 1.0f);
    set_mat8(msg.mutable_fmat8(), 1.0f);
    set_mat9(msg.mutable_fmat9(), 1.0f);
    set_mat10(msg.mutable_fmat10(), 1.0f);
    set_mat11(msg.mutable_fmat11(), 1.0f);
    set_mat12(msg.mutable_fmat12(), 1.0f);
    set_mat13(msg.mutable_fmat13(), 1.0f);
    set_mat14(msg.mutable_fmat14(), 1.0f);
    set_mat15(msg.mutable_fmat15(), 1.0f);
    set_mat16(msg.mutable_fmat16(), 1.0f);

    set_mat2(msg.mutable_imat2(), 1);
    set_mat3(msg.mutable_imat3(), 1);
    set_mat4(msg.mutable_imat4(), 1);
    set_mat5(msg.mutable_imat5(), 1);
    set_mat6(msg.mutable_imat6(), 1);
    set_mat7(msg.mutable_imat7(), 1);
    set_mat8(msg.mutable_imat8(), 1);
    set_mat9(msg.mutable_imat9(), 1);
    set_mat10(msg.mutable_imat10(), 1);
    set_mat11(msg.mutable_imat11(), 1);
    set_mat12(msg.mutable_imat12(), 1);
    set_mat13(msg.mutable_imat13(), 1);
    set_mat14(msg.mutable_imat14(), 1);
    set_mat15(msg.mutable_imat15(), 1);
    set_mat16(msg.mutable_imat16(), 1);

    set_mat2(msg.mutable_umat2(), 1u);
    set_mat3(msg.mutable_umat3(), 1u);
    set_mat4(msg.mutable_umat4(), 1u);
    set_mat5(msg.mutable_umat5(), 1u);
    set_mat6(msg.mutable_umat6(), 1u);
    set_mat7(msg.mutable_umat7(), 1u);
    set_mat8(msg.mutable_umat8(), 1u);
    set_mat9(msg.mutable_umat9(), 1u);
    set_mat10(msg.mutable_umat10(), 1u);
    set_mat11(msg.mutable_umat11(), 1u);
    set_mat12(msg.mutable_umat12(), 1u);
    set_mat13(msg.mutable_umat13(), 1u);
    set_mat14(msg.mutable_umat14(), 1u);
    set_mat15(msg.mutable_umat15(), 1u);
    set_mat16(msg.mutable_umat16(), 1u);

    // Populate quaternions
    set_quat(msg.mutable_quat(), 1.0);
    set_quat(msg.mutable_fquat(), 1.0f);

    // Populate isometries
    set_mat3(msg.mutable_iso2(), 1.0);
    set_mat3(msg.mutable_fiso2(), 1.0f);
    set_mat4(msg.mutable_iso3(), 1.0);
    set_mat4(msg.mutable_fiso3(), 1.0f);

    return msg;
}

MessageTest construct_message(const NUClear::clock::time_point& tp) {
    MessageTest msg;

    // Populate timestamp and duration on message
    msg.timestamp = tp;
    msg.duration  = tp.time_since_epoch();

    // Populate scalar field values
    msg.d        = 3.0;
    msg.f        = 4.0;
    msg.i32      = 5;
    msg.i64      = 6;
    msg.u32      = 7;
    msg.u64      = 8;
    msg.s32      = 9;
    msg.s64      = 10;
    msg.fixed32  = 11;
    msg.fixed64  = 12;
    msg.sfixed32 = 13;
    msg.sfixed64 = 14;
    msg.b        = true;
    msg.s        = "16";
    msg.bytes    = {'1', '7'};

    // Populate dynamic vector fields
    msg.vec.resize(5);
    msg.fvec.resize(5);
    msg.ivec.resize(5);
    msg.uvec.resize(5);
    msg.cvec.resize(5);
    for (int i = 0; i < 5; ++i) {
        msg.vec[i]  = double(i + 1);
        msg.fvec[i] = float(i + 1);
        msg.ivec[i] = i + 1;
        msg.uvec[i] = i + 1;
        msg.cvec[i] = char(i + 1);
    }

    // Populate dynamic matrix fields
    msg.mat  = ::message::conversion::math::mat(5, 5);
    msg.fmat = ::message::conversion::math::fmat(5, 5);
    msg.imat = ::message::conversion::math::imat(5, 5);
    msg.umat = ::message::conversion::math::umat(5, 5);
    msg.cmat = ::message::conversion::math::cmat(5, 5);
    for (int c = 0; c < 5; ++c) {
        for (int r = 0; r < 5; ++r) {
            msg.mat(r, c)  = double(5 * c + r + 1);
            msg.fmat(r, c) = float(5 * c + r + 1);
            msg.imat(r, c) = 5 * c + r + 1;
            msg.umat(r, c) = 5 * c + r + 1;
            msg.cmat(r, c) = char(5 * c + r + 1);
        }
    }

    // Populate fixed sized vectors
    // clang-format off
    set(msg.vec2);  set(msg.fvec2);  set(msg.ivec2);  set(msg.uvec2);
    set(msg.vec3);  set(msg.fvec3);  set(msg.ivec3);  set(msg.uvec3);
    set(msg.vec4);  set(msg.fvec4);  set(msg.ivec4);  set(msg.uvec4);
    set(msg.vec5);  set(msg.fvec5);  set(msg.ivec5);  set(msg.uvec5);
    set(msg.vec6);  set(msg.fvec6);  set(msg.ivec6);  set(msg.uvec6);
    set(msg.vec7);  set(msg.fvec7);  set(msg.ivec7);  set(msg.uvec7);
    set(msg.vec8);  set(msg.fvec8);  set(msg.ivec8);  set(msg.uvec8);
    set(msg.vec9);  set(msg.fvec9);  set(msg.ivec9);  set(msg.uvec9);
    set(msg.vec10); set(msg.fvec10); set(msg.ivec10); set(msg.uvec10);
    set(msg.vec11); set(msg.fvec11); set(msg.ivec11); set(msg.uvec11);
    set(msg.vec12); set(msg.fvec12); set(msg.ivec12); set(msg.uvec12);
    set(msg.vec13); set(msg.fvec13); set(msg.ivec13); set(msg.uvec13);
    set(msg.vec14); set(msg.fvec14); set(msg.ivec14); set(msg.uvec14);
    set(msg.vec15); set(msg.fvec15); set(msg.ivec15); set(msg.uvec15);
    set(msg.vec16); set(msg.fvec16); set(msg.ivec16); set(msg.uvec16);
    // clang-format on

    // Populate fixed sized matrices
    // clang-format off
    set(msg.mat2);  set(msg.fmat2);  set(msg.imat2);  set(msg.umat2);
    set(msg.mat3);  set(msg.fmat3);  set(msg.imat3);  set(msg.umat3);
    set(msg.mat4);  set(msg.fmat4);  set(msg.imat4);  set(msg.umat4);
    set(msg.mat5);  set(msg.fmat5);  set(msg.imat5);  set(msg.umat5);
    set(msg.mat6);  set(msg.fmat6);  set(msg.imat6);  set(msg.umat6);
    set(msg.mat7);  set(msg.fmat7);  set(msg.imat7);  set(msg.umat7);
    set(msg.mat8);  set(msg.fmat8);  set(msg.imat8);  set(msg.umat8);
    set(msg.mat9);  set(msg.fmat9);  set(msg.imat9);  set(msg.umat9);
    set(msg.mat10); set(msg.fmat10); set(msg.imat10); set(msg.umat10);
    set(msg.mat11); set(msg.fmat11); set(msg.imat11); set(msg.umat11);
    set(msg.mat12); set(msg.fmat12); set(msg.imat12); set(msg.umat12);
    set(msg.mat13); set(msg.fmat13); set(msg.imat13); set(msg.umat13);
    set(msg.mat14); set(msg.fmat14); set(msg.imat14); set(msg.umat14);
    set(msg.mat15); set(msg.fmat15); set(msg.imat15); set(msg.umat15);
    set(msg.mat16); set(msg.fmat16); set(msg.imat16); set(msg.umat16);
    // clang-format on

    // Populate quaternions
    set(msg.quat.coeffs());
    set(msg.fquat.coeffs());

    // Populate isometries
    set(msg.iso2.matrix());
    set(msg.fiso2.matrix());
    set(msg.iso3.matrix());
    set(msg.fiso3.matrix());

    return msg;
}

SCENARIO("Conversion from protobuf message to Neutron is consistent", "[nuclear][message][converter]") {
    GIVEN("A protobuf message in a raw binary format") {
        // Get the current time so we can use the same time in both message formats
        const auto tp = NUClear::clock::now();

        // Construct a protobuf message and fill it with some data
        PbMessageTest pb_msg = construct_pb_message(tp);

        // Construct a neutron message and fill it with the same data
        MessageTest nt_msg = construct_message(tp);

        WHEN("A protobuf message is converted to a neutron") {
            auto msg = MessageTest(pb_msg);
            THEN("they are equal") {
                REQUIRE(nt_msg == msg);
            }
        }

        WHEN("A neutron message is converted to a protobuf") {
            auto msg = PbMessageTest(nt_msg);
            THEN("they are equal") {
                REQUIRE((pb_msg.timestamp().seconds() == msg.timestamp().seconds()
                         && pb_msg.timestamp().nanos() == msg.timestamp().nanos()));
                REQUIRE((pb_msg.duration().seconds() == msg.duration().seconds()
                         && pb_msg.duration().nanos() == msg.duration().nanos()));
                REQUIRE(pb_msg.d() == msg.d());
                REQUIRE(pb_msg.f() == msg.f());
                REQUIRE(pb_msg.i32() == msg.i32());
                REQUIRE(pb_msg.i64() == msg.i64());
                REQUIRE(pb_msg.u32() == msg.u32());
                REQUIRE(pb_msg.u64() == msg.u64());
                REQUIRE(pb_msg.s32() == msg.s32());
                REQUIRE(pb_msg.s64() == msg.s64());
                REQUIRE(pb_msg.fixed32() == msg.fixed32());
                REQUIRE(pb_msg.fixed64() == msg.fixed64());
                REQUIRE(pb_msg.sfixed32() == msg.sfixed32());
                REQUIRE(pb_msg.sfixed64() == msg.sfixed64());
                REQUIRE(pb_msg.b() == msg.b());
                REQUIRE(pb_msg.s() == msg.s());
                REQUIRE(pb_msg.bytes() == msg.bytes());
                REQUIRE(check_mat3(pb_msg.iso2(), msg.iso2()));
                REQUIRE(check_mat3(pb_msg.fiso2(), msg.fiso2()));
                REQUIRE(check_mat4(pb_msg.iso3(), msg.iso3()));
                REQUIRE(check_mat4(pb_msg.fiso3(), msg.fiso3()));
                REQUIRE(check_quat(pb_msg.quat(), msg.quat()));
                REQUIRE(check_quat(pb_msg.fquat(), msg.fquat()));
                REQUIRE(check_vec(pb_msg.vec(), msg.vec()));
                REQUIRE(check_vec(pb_msg.fvec(), msg.fvec()));
                REQUIRE(check_vec(pb_msg.ivec(), msg.ivec()));
                REQUIRE(check_vec(pb_msg.uvec(), msg.uvec()));
                REQUIRE(check_vec(pb_msg.cvec(), msg.cvec()));
                REQUIRE(check_vec2(pb_msg.vec2(), msg.vec2()));
                REQUIRE(check_vec2(pb_msg.fvec2(), msg.fvec2()));
                REQUIRE(check_vec2(pb_msg.ivec2(), msg.ivec2()));
                REQUIRE(check_vec2(pb_msg.uvec2(), msg.uvec2()));
                REQUIRE(check_vec3(pb_msg.vec3(), msg.vec3()));
                REQUIRE(check_vec3(pb_msg.fvec3(), msg.fvec3()));
                REQUIRE(check_vec3(pb_msg.ivec3(), msg.ivec3()));
                REQUIRE(check_vec3(pb_msg.uvec3(), msg.uvec3()));
                REQUIRE(check_vec4(pb_msg.vec4(), msg.vec4()));
                REQUIRE(check_vec4(pb_msg.fvec4(), msg.fvec4()));
                REQUIRE(check_vec4(pb_msg.ivec4(), msg.ivec4()));
                REQUIRE(check_vec4(pb_msg.uvec4(), msg.uvec4()));
                REQUIRE(check_vec5(pb_msg.vec5(), msg.vec5()));
                REQUIRE(check_vec5(pb_msg.fvec5(), msg.fvec5()));
                REQUIRE(check_vec5(pb_msg.ivec5(), msg.ivec5()));
                REQUIRE(check_vec5(pb_msg.uvec5(), msg.uvec5()));
                REQUIRE(check_vec6(pb_msg.vec6(), msg.vec6()));
                REQUIRE(check_vec6(pb_msg.fvec6(), msg.fvec6()));
                REQUIRE(check_vec6(pb_msg.ivec6(), msg.ivec6()));
                REQUIRE(check_vec6(pb_msg.uvec6(), msg.uvec6()));
                REQUIRE(check_vec7(pb_msg.vec7(), msg.vec7()));
                REQUIRE(check_vec7(pb_msg.fvec7(), msg.fvec7()));
                REQUIRE(check_vec7(pb_msg.ivec7(), msg.ivec7()));
                REQUIRE(check_vec7(pb_msg.uvec7(), msg.uvec7()));
                REQUIRE(check_vec8(pb_msg.vec8(), msg.vec8()));
                REQUIRE(check_vec8(pb_msg.fvec8(), msg.fvec8()));
                REQUIRE(check_vec8(pb_msg.ivec8(), msg.ivec8()));
                REQUIRE(check_vec8(pb_msg.uvec8(), msg.uvec8()));
                REQUIRE(check_vec9(pb_msg.vec9(), msg.vec9()));
                REQUIRE(check_vec9(pb_msg.fvec9(), msg.fvec9()));
                REQUIRE(check_vec9(pb_msg.ivec9(), msg.ivec9()));
                REQUIRE(check_vec9(pb_msg.uvec9(), msg.uvec9()));
                REQUIRE(check_vec10(pb_msg.vec10(), msg.vec10()));
                REQUIRE(check_vec10(pb_msg.fvec10(), msg.fvec10()));
                REQUIRE(check_vec10(pb_msg.ivec10(), msg.ivec10()));
                REQUIRE(check_vec10(pb_msg.uvec10(), msg.uvec10()));
                REQUIRE(check_vec11(pb_msg.vec11(), msg.vec11()));
                REQUIRE(check_vec11(pb_msg.fvec11(), msg.fvec11()));
                REQUIRE(check_vec11(pb_msg.ivec11(), msg.ivec11()));
                REQUIRE(check_vec11(pb_msg.uvec11(), msg.uvec11()));
                REQUIRE(check_vec12(pb_msg.vec12(), msg.vec12()));
                REQUIRE(check_vec12(pb_msg.fvec12(), msg.fvec12()));
                REQUIRE(check_vec12(pb_msg.ivec12(), msg.ivec12()));
                REQUIRE(check_vec12(pb_msg.uvec12(), msg.uvec12()));
                REQUIRE(check_vec13(pb_msg.vec13(), msg.vec13()));
                REQUIRE(check_vec13(pb_msg.fvec13(), msg.fvec13()));
                REQUIRE(check_vec13(pb_msg.ivec13(), msg.ivec13()));
                REQUIRE(check_vec13(pb_msg.uvec13(), msg.uvec13()));
                REQUIRE(check_vec14(pb_msg.vec14(), msg.vec14()));
                REQUIRE(check_vec14(pb_msg.fvec14(), msg.fvec14()));
                REQUIRE(check_vec14(pb_msg.ivec14(), msg.ivec14()));
                REQUIRE(check_vec14(pb_msg.uvec14(), msg.uvec14()));
                REQUIRE(check_vec15(pb_msg.vec15(), msg.vec15()));
                REQUIRE(check_vec15(pb_msg.fvec15(), msg.fvec15()));
                REQUIRE(check_vec15(pb_msg.ivec15(), msg.ivec15()));
                REQUIRE(check_vec15(pb_msg.uvec15(), msg.uvec15()));
                REQUIRE(check_vec16(pb_msg.vec16(), msg.vec16()));
                REQUIRE(check_vec16(pb_msg.fvec16(), msg.fvec16()));
                REQUIRE(check_vec16(pb_msg.ivec16(), msg.ivec16()));
                REQUIRE(check_vec16(pb_msg.uvec16(), msg.uvec16()));
                REQUIRE(check_mat(pb_msg.mat(), msg.mat()));
                REQUIRE(check_mat(pb_msg.fmat(), msg.fmat()));
                REQUIRE(check_mat(pb_msg.imat(), msg.imat()));
                REQUIRE(check_mat(pb_msg.umat(), msg.umat()));
                REQUIRE(check_mat(pb_msg.cmat(), msg.cmat()));
                REQUIRE(check_mat2(pb_msg.mat2(), msg.mat2()));
                REQUIRE(check_mat2(pb_msg.fmat2(), msg.fmat2()));
                REQUIRE(check_mat2(pb_msg.imat2(), msg.imat2()));
                REQUIRE(check_mat2(pb_msg.umat2(), msg.umat2()));
                REQUIRE(check_mat3(pb_msg.mat3(), msg.mat3()));
                REQUIRE(check_mat3(pb_msg.fmat3(), msg.fmat3()));
                REQUIRE(check_mat3(pb_msg.imat3(), msg.imat3()));
                REQUIRE(check_mat3(pb_msg.umat3(), msg.umat3()));
                REQUIRE(check_mat4(pb_msg.mat4(), msg.mat4()));
                REQUIRE(check_mat4(pb_msg.fmat4(), msg.fmat4()));
                REQUIRE(check_mat4(pb_msg.imat4(), msg.imat4()));
                REQUIRE(check_mat4(pb_msg.umat4(), msg.umat4()));
                REQUIRE(check_mat5(pb_msg.mat5(), msg.mat5()));
                REQUIRE(check_mat5(pb_msg.fmat5(), msg.fmat5()));
                REQUIRE(check_mat5(pb_msg.imat5(), msg.imat5()));
                REQUIRE(check_mat5(pb_msg.umat5(), msg.umat5()));
                REQUIRE(check_mat6(pb_msg.mat6(), msg.mat6()));
                REQUIRE(check_mat6(pb_msg.fmat6(), msg.fmat6()));
                REQUIRE(check_mat6(pb_msg.imat6(), msg.imat6()));
                REQUIRE(check_mat6(pb_msg.umat6(), msg.umat6()));
                REQUIRE(check_mat7(pb_msg.mat7(), msg.mat7()));
                REQUIRE(check_mat7(pb_msg.fmat7(), msg.fmat7()));
                REQUIRE(check_mat7(pb_msg.imat7(), msg.imat7()));
                REQUIRE(check_mat7(pb_msg.umat7(), msg.umat7()));
                REQUIRE(check_mat8(pb_msg.mat8(), msg.mat8()));
                REQUIRE(check_mat8(pb_msg.fmat8(), msg.fmat8()));
                REQUIRE(check_mat8(pb_msg.imat8(), msg.imat8()));
                REQUIRE(check_mat8(pb_msg.umat8(), msg.umat8()));
                REQUIRE(check_mat9(pb_msg.mat9(), msg.mat9()));
                REQUIRE(check_mat9(pb_msg.fmat9(), msg.fmat9()));
                REQUIRE(check_mat9(pb_msg.imat9(), msg.imat9()));
                REQUIRE(check_mat9(pb_msg.umat9(), msg.umat9()));
                REQUIRE(check_mat10(pb_msg.mat10(), msg.mat10()));
                REQUIRE(check_mat10(pb_msg.fmat10(), msg.fmat10()));
                REQUIRE(check_mat10(pb_msg.imat10(), msg.imat10()));
                REQUIRE(check_mat10(pb_msg.umat10(), msg.umat10()));
                REQUIRE(check_mat11(pb_msg.mat11(), msg.mat11()));
                REQUIRE(check_mat11(pb_msg.fmat11(), msg.fmat11()));
                REQUIRE(check_mat11(pb_msg.imat11(), msg.imat11()));
                REQUIRE(check_mat11(pb_msg.umat11(), msg.umat11()));
                REQUIRE(check_mat12(pb_msg.mat12(), msg.mat12()));
                REQUIRE(check_mat12(pb_msg.fmat12(), msg.fmat12()));
                REQUIRE(check_mat12(pb_msg.imat12(), msg.imat12()));
                REQUIRE(check_mat12(pb_msg.umat12(), msg.umat12()));
                REQUIRE(check_mat13(pb_msg.mat13(), msg.mat13()));
                REQUIRE(check_mat13(pb_msg.fmat13(), msg.fmat13()));
                REQUIRE(check_mat13(pb_msg.imat13(), msg.imat13()));
                REQUIRE(check_mat13(pb_msg.umat13(), msg.umat13()));
                REQUIRE(check_mat14(pb_msg.mat14(), msg.mat14()));
                REQUIRE(check_mat14(pb_msg.fmat14(), msg.fmat14()));
                REQUIRE(check_mat14(pb_msg.imat14(), msg.imat14()));
                REQUIRE(check_mat14(pb_msg.umat14(), msg.umat14()));
                REQUIRE(check_mat15(pb_msg.mat15(), msg.mat15()));
                REQUIRE(check_mat15(pb_msg.fmat15(), msg.fmat15()));
                REQUIRE(check_mat15(pb_msg.imat15(), msg.imat15()));
                REQUIRE(check_mat15(pb_msg.umat15(), msg.umat15()));
                REQUIRE(check_mat16(pb_msg.mat16(), msg.mat16()));
                REQUIRE(check_mat16(pb_msg.fmat16(), msg.fmat16()));
                REQUIRE(check_mat16(pb_msg.imat16(), msg.imat16()));
                REQUIRE(check_mat16(pb_msg.umat16(), msg.umat16()));
            }
        }
    }
}
