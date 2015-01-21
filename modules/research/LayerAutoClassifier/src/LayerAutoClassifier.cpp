/*
 * This file is part of NUbots Codebase.
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

#include "LayerAutoClassifier.h"

#include "messages/research/AutoClassifierPixels.h"
#include "messages/vision/LookUpTable.h"

namespace modules {
namespace research {

    using messages::input::Image;
    using messages::research::AutoClassifierPixels;
    using messages::vision::LookUpTable;
    using messages::vision::Colour;


    inline int getIndex(const LookUpTable& lut, uint8_t x, uint8_t y, uint8_t z) {
        return (((x << lut.BITS_Y) | y) << lut.BITS_CB) | z;
    }

    inline Colour getAt(const LookUpTable& lut, uint8_t x, uint8_t y, uint8_t z) {
        return lut.getRawData()[getIndex(lut, x, y, z)];
    }

    inline std::array<Colour, 6> getSurrounding(const LookUpTable& lut, uint8_t x, uint8_t y, uint8_t z) {
        return {
            getAt(lut, x + 1, y, z),
            getAt(lut, x - 1, y, z),
            getAt(lut, x, y + 1, z),
            getAt(lut, x, y - 1, z),
            getAt(lut, x, y, z + 1),
            getAt(lut, x, y, z - 1)
        };
    }

    inline bool isSA(const LookUpTable& lut, uint8_t x, uint8_t y, uint8_t z) {

        // Get our colour
        Colour us = getAt(lut, x, y, z);
        // Get surrounding colours
        auto surrounds = getSurrounding(lut, x, y, z);

        // If we are a surface area voxel (two adjacent voxels)
        return !((us == surrounds[0] && surrounds[0] == surrounds[1])
                || (us == surrounds[2] && surrounds[2] == surrounds[3])
                || (us == surrounds[4] && surrounds[4] == surrounds[5]));

    }

    LayerAutoClassifier::LayerAutoClassifier(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        // When we get a look up table then it was uploaded or emitted by someone else
        // We need to set it up for our datastructure
        on<Trigger<LookUpTable>>([this] (const LookUpTable& lut) {

            std::map<Colour, std::set<std::array<int, 3>>> sa;
            std::map<Colour, int> vol;

            for(int x = 0; x < (1 << lut.BITS_Y); ++x) {
                for (int y = 0; y < (1 << lut.BITS_CB); ++y) {
                    for (int z = 0; z < (1 << lut.BITS_CR); ++z) {

                        // Get our classification
                        Colour us = lut.getRawData()[(((x << lut.BITS_Y) | y) << lut.BITS_CB) | z];

                        // Ignore unclassified pixels
                        if(us != Colour::UNCLASSIFIED) {

                            // Increase our volume
                            ++vol[us];

                            // If this is a surface area pixel add it to the list
                            if(isSA(lut, x, y, z)) {
                                sa[us].insert({x, y, z});
                            }
                        }
                    }
                }
            }

            for(auto& v : vol) {
                log("Volume:", char(v.first), v.second);
            }

            for(auto& p : sa) {
                for(auto& i : p.second) {
                    log("SA:", char(p.first), i[0], i[1], i[2]);
                }
            }
        });


        	for(auto& pixel : pixels.pixels) {

        		// Do the algorithm!!
        	}
        });
    }

}
}

