#ifndef TIMER_HPP
#define TIMER_HPP

#include <chrono>

class Timer {
public:
    std::chrono::steady_clock::time_point t;

    Timer() : t(std::chrono::steady_clock::now()) {}

    template <size_t N>
    inline void measure(const char (&c)[N]) {
        auto end = std::chrono::steady_clock::now();

        auto val = end - t;

        auto v = std::chrono::duration_cast<std::chrono::duration<uint64_t, std::micro>>(val).count();

        std::cout << c << " " << v << "µs" << std::endl;

        t = std::chrono::steady_clock::now();
    }

    inline void reset() {
        t = std::chrono::steady_clock::now();
    }
};

#endif  // TIMER_HPP
