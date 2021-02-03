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

        float pathlength(float x, float y, float h) {
            float dx     = 0;
            float dy     = 0;
            float norm   = 0;
            float dxnorm = 0;
            float dynorm = 0;
            float p      = 0;

            while (y > 0) {
                dx     = vx(x);
                dy     = vy(y, x, h);
                norm   = sqrt(dx * dx + dy * dy);  // normalise the vector to length 1
                dxnorm = dx / norm;                // move x and y a small amount in that direction
                dynorm = dy / norm;                // add the hypotenuse to the path length variable
                x      = x + dxnorm * 0.01;
                y      = y + dynorm * 0.01;
                p      = p + 0.01414;
            }

            printf("Aproximate path length is %f", p);

            return p;
        }
    }  // namespace walk
}  // namespace motion
}  // namespace utility
