/*
 * Copyright (C) 2017-2018 Trent Houliston <trent@houliston.me>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

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
      if (n > 0) { out << s[j][n - 1]; }
      out << "]";

      if (j < m - 1) { out << std::endl; }
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
    if (n > 0) { out << s[n - 1]; }
    out << "]";
  }
};

template <typename T, std::size_t n>
std::ostream& operator<<(std::ostream& out, const std::array<T, n>& s) {
  Printer<std::array<T, n>>::print(out, s);
  return out;
}

#endif  // ARRAY_PRINT_HPP
