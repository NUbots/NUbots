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

/**
 *       @name LookUpTable
 *       @file lookuptable.h
 *       @brief Wraps LUT buffer with access methods for pixel classification
 *       @author Shannon Fenn
 *       @date 17-02-12
 *       @note ported by Jake Fountain Dec 2013 to NUClear system
 */
#ifndef UTILITY_VISION_LOOKUPTABLE_H
#define UTILITY_VISION_LOOKUPTABLE_H

#include <yaml-cpp/yaml.h>
#include <cmath>
#include <cstring>

#include "message/vision/LookUpTable.h"

#include "Vision.h"

namespace utility {
namespace vision {

    /*!
     *   @brief Gets the index of the pixel in the LUT
     *   @param p The pixel to be classified.
     *   @return Returns the colour index for the given pixel.
     */
    static inline uint32_t getLUTIndex(const message::vision::LookUpTable& lut, const Pixel& p) {
        const uint8_t BITS_Y_REMOVED  = sizeof(uint8_t) * 8 - lut.bits_y;
        const uint8_t BITS_CB_REMOVED = sizeof(uint8_t) * 8 - lut.bits_cb;
        const uint8_t BITS_CR_REMOVED = sizeof(uint8_t) * 8 - lut.bits_cr;
        const uint8_t BITS_CB_CR      = lut.bits_cb + lut.bits_cr;

        uint32_t index = 0;

        index += ((p.components.y >> BITS_Y_REMOVED) << BITS_CB_CR);
        index += ((p.components.cb >> BITS_CB_REMOVED) << lut.bits_cr);
        index += (p.components.cr >> BITS_CR_REMOVED);

        return index;
    }

    /*!
        @brief Classifies a pixel
        @param p the pixel
        @return Returns the colour classification of this pixel
     */
    static inline Colour getPixelColour(const message::vision::LookUpTable& lut, const Pixel& p) {
        return lut.table[getLUTIndex(lut, p)];
    }

    /*!
     *   @brief The inverse of getLUTIndex
     *   NOTE: This inverse is NOT injective (e.g. not 1-to-1)
     */
    static inline Pixel getPixelFromIndex(const message::vision::LookUpTable& lut, const uint& index) {
        const uint8_t BITS_Y_REMOVED  = sizeof(uint8_t) * 8 - lut.bits_y;
        const uint8_t BITS_CB_REMOVED = sizeof(uint8_t) * 8 - lut.bits_cb;
        const uint8_t BITS_CR_REMOVED = sizeof(uint8_t) * 8 - lut.bits_cr;
        const uint8_t BITS_CB_CR      = lut.bits_cb + lut.bits_cr;
        const uint8_t BITS_CB_MASK = (1 << lut.bits_cb) - 1;  // std::pow(2, lut.bits_cb) - 1;
        const uint8_t BITS_CR_MASK = (1 << lut.bits_cr) - 1;  // std::pow(2, lut.bits_cr) - 1;

        uint8_t y  = (index >> BITS_CB_CR) << BITS_Y_REMOVED;
        uint8_t cb = ((index >> lut.bits_cr) & BITS_CB_MASK) << BITS_CB_REMOVED;
        uint8_t cr = (index & BITS_CR_MASK) << BITS_CR_REMOVED;

        return Pixel(static_cast<uint32_t>((y << 16) | (cb << 8) | cr));
    }

}  // vision
}  // utility

// YAML conversions
namespace YAML {

template <>
struct convert<message::vision::LookUpTable> {
    static Node encode(const message::vision::LookUpTable& rhs) {
        Node node;

        node["bits"]["y"]  = uint(rhs.bits_y);
        node["bits"]["cb"] = uint(rhs.bits_cb);
        node["bits"]["cr"] = uint(rhs.bits_cr);

        node["lut"] = std::string(rhs.table.begin(), rhs.table.end());

        return node;
    }

    static bool decode(const Node& node, message::vision::LookUpTable& rhs) {
        uint8_t bitsY  = node["bits"]["y"].as<uint>();
        uint8_t bitsCb = node["bits"]["cb"].as<uint>();
        uint8_t bitsCr = node["bits"]["cr"].as<uint>();

        std::string dataString = node["lut"].as<std::string>();
        std::vector<uint8_t> data;
        data.reserve(dataString.size());

        for (auto& s : dataString) {
            data.push_back(static_cast<uint8_t>(s));
        }

        rhs = message::vision::LookUpTable(std::move(data), bitsY, bitsCb, bitsCr);

        return true;
    }
};
}

#endif  // UTILITY_VISION_LOOKUPTABLE_H
