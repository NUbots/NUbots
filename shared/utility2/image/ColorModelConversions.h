/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef SHARED_UTILITY_IMAGE_COLORMODELCONVERSIONS_H
#define SHARED_UTILITY_IMAGE_COLORMODELCONVERSIONS_H
#include "RGB.h"
#include "HSV.h"
#include "YCbCr.h"
namespace utility {
    /**
     * TODO document
     *
     * @author Jake Woods
     */
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
