// Savitzky-Golay filter
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef UTILITY_MATH_FILTER_GOLAY_H
#define UTILITY_MATH_FILTER_GOLAY_H

#include <boost/circular_buffer.hpp>
#include <type_traits>

namespace utility {
namespace math {
    namespace filter {

        template <int DerivGrade, int WindowSize>
        class GolayCoeff {};

        template <>
        class GolayCoeff<0, 1> {
        public:
            constexpr static double Coefficients[] = {1};
            constexpr static double Normalization  = 1;
        };

        template <>
        class GolayCoeff<0, 5> {
        public:
            constexpr static double Coefficients[] = {-3, 12, 17, 12, -3};
            constexpr static double Normalization  = 35;
        };

        template <>
        class GolayCoeff<0, 7> {
        public:
            constexpr static double Coefficients[] = {-2, 3, 6, 7, 6, 3, -2};
            constexpr static double Normalization  = 21;
        };

        template <>
        class GolayCoeff<0, 9> {
        public:
            constexpr static double Coefficients[] = {-21, 14, 39, 54, 59, 54, 39, 14, -21};
            constexpr static double Normalization  = 231;
        };

        template <>
        class GolayCoeff<1, 5> {
        public:
            constexpr static double Coefficients[] = {-2, -1, 0, 1, 2};
            constexpr static double Normalization  = 10;
        };

        template <>
        class GolayCoeff<1, 7> {
        public:
            constexpr static double Coefficients[] = {-3, -2, -1, 0, 1, 2, 3};
            constexpr static double Normalization  = 28;
        };

        template <>
        class GolayCoeff<1, 9> {
        public:
            constexpr static double Coefficients[] = {-4, -3, -2, -1, 0, 1, 2, 3, 4};
            constexpr static double Normalization  = 60;
        };

        template <>
        class GolayCoeff<2, 5> {
        public:
            constexpr static double Coefficients[] = {2, -1, -2, -1, 2};
            constexpr static double Normalization  = 7;
        };

        template <>
        class GolayCoeff<2, 7> {
        public:
            constexpr static double Coefficients[] = {5, 0, -3, -4, -3, 0, 5};
            constexpr static double Normalization  = 42;
        };

        template <>
        class GolayCoeff<2, 9> {
        public:
            constexpr static double Coefficients[] = {28, 7, -8, -17, -20, -17, -8, 7, 28};
            constexpr static double Normalization  = 462;
        };


        template <class T, int DerivGrade, int WindowSize = 9, class Alloc = std::allocator<T>>
        class GolayDerivative {
        private:
            template <typename Q>
            struct InitValue {
                inline static Q init() {
                    return Q(0);
                }
            };

// Eigen3 type support
#ifdef EIGEN_CORE_H
            template <class Q, int Rows, int Cols>
            struct InitValue<Eigen::Matrix<Q, Rows, Cols>> {
                inline static Eigen::Matrix<Q, Rows, Cols> init() {
                    return Eigen::Matrix<Q, Rows, Cols>::Zero();
                }
            };
#endif

        public:
            GolayDerivative() : m_buf(WindowSize){};

            typedef boost::circular_buffer<T, Alloc> BufferType;

            void put(const T& data) {
                m_buf.push_back(data);
            }

            T value() const {
                T val = InitValue<T>::init();

                if (m_buf.size() != WindowSize) return val;

                for (size_t i = 0; i < WindowSize; ++i)
                    val += GolayCoeff<DerivGrade, WindowSize>::Coefficients[i] * m_buf[i];

                return val / GolayCoeff<DerivGrade, WindowSize>::Normalization;
            };

            BufferType* buffer() {
                return &m_buf;
            }

            inline void reset() {
                m_buf.clear();
            }

        private:
            BufferType m_buf;
        };

    }  // namespace filter
}  // namespace math
}  // namespace utility

#endif  // UTILITY_MATH_FILTER_GOLAY_H
