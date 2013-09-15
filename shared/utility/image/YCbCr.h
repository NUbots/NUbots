/*
 * This file is part of NUBots Utility.
 *
 * NUBots Utility is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * NUBots Utility is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with NUBots Utility.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef UTILITY_IMAGE_YCBCR_H
#define UTILITY_IMAGE_YCBCR_H
namespace utility {
namespace image {
    
    /**
     * TODO document
     *
     * @author Jake Woods
     */
    struct YCbCr {
        public:
            unsigned char Y;
            unsigned char Cb;
            unsigned char Cr;
    };
}
}
#endif
