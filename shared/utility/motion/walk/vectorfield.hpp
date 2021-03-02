#include <cmath>
#include <cstdio>
#include <iostream>
using namespace std;
namespace utility {
namespace motion {
    namespace walk {
        float vx(float x) {
            return -tanh(x);
        }

        float vy(float y, float x, float h) {
            return h * tanh(x) * tanh(x) - y;
        }

    }  // namespace walk
}  // namespace motion
}  // namespace utility
