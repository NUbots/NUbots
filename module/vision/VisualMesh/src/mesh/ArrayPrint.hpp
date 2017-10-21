#ifndef ARRAY_PRINT_HPP
#define ARRAY_PRINT_HPP

#include <array>
#include <iostream>

template <typename T>
struct Printer;

// Print a matrix
template <typename Scalar, std::size_t n, std::size_t m>
struct Printer<std::array<std::array<Scalar, n>, m>> {
    static inline void print(std::ostream& out, const std::array<std::array<Scalar, n>, m>& s) {
        for (std::size_t j = 0; j < m; ++j) {
            out << "[";
            for (std::size_t i = 0; i < n - 1; ++i) {
                out << s[j][i] << ", ";
            }
            if (n > 0) {
                out << s[j][n - 1];
            }
            out << "]";

            if (j < m - 1) {
                out << std::endl;
            }
        }
    }
};

// Print a vector
template <typename Scalar, std::size_t n>
struct Printer<std::array<Scalar, n>> {
    static inline void print(std::ostream& out, const std::array<Scalar, n>& s) {
        out << "[";
        for (std::size_t i = 0; i < n - 1; ++i) {
            out << s[i] << ", ";
        }
        if (n > 0) {
            out << s[n - 1];
        }
        out << "]";
    }
};

template <typename T, std::size_t n>
std::ostream& operator<<(std::ostream& out, const std::array<T, n>& s) {
    Printer<std::array<T, n>>::print(out, s);
    return out;
}

#endif  // ARRAY_PRINT_HPP
