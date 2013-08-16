#ifndef SHARED_UTILITY_IMAGE_COLORMODELCONVERSIONS_H
#define SHARED_UTILITY_IMAGE_COLORMODELCONVERSIONS_H
#include "RGB.h"
#include "HSV.h"
#include "YCbCr.h"
namespace utility {
namespace image {
     RGB toRGB(YCbCr ycbcr);
     RGB toRGB(HSV hsv);
     HSV toHSV(RGB rgb);
     HSV toHSV(YCbCr ycbcr);
     YCbCr toYCbCr(RGB rgb);
     YCbCr toYCbCr(HSV hsv);
}
}
#endif
