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
#include "messages/support/Configuration.h"
#include "messages/vision/LookUpTable.h"
#include "messages/vision/proto/LookUpTable.pb.h"

namespace modules {
namespace research {

    using messages::input::Image;
    using messages::research::AutoClassifierPixels;
    using messages::support::Configuration;
    using messages::vision::LookUpTable;
    using messages::vision::proto::LookUpTableDiff;
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

    void shed(LookUpTable& lut, Colour c, std::set<std::array<int, 3>>& sa, int& vol) {
        // Holds our new surface voxels
        std::set<std::array<int, 3>> newSA;

        // Go through the surface area for this colour
        for(auto& s : sa) {

            // Get the surrounds as they may be new SA voxels
            auto surrounds = getSurrounding(lut, s[0], s[1], s[2]);

            // Insert these into our potential new SA points
            if(surrounds[0] == c) newSA.insert({ s[0] + 1, s[1], s[2] });
            if(surrounds[1] == c) newSA.insert({ s[0] - 1, s[1], s[2] });
            if(surrounds[2] == c) newSA.insert({ s[0], s[1] + 1, s[2] });
            if(surrounds[3] == c) newSA.insert({ s[0], s[1] - 1, s[2] });
            if(surrounds[4] == c) newSA.insert({ s[0], s[1], s[2] + 1 });
            if(surrounds[5] == c) newSA.insert({ s[0], s[1], s[2] - 1 });

            // Set this voxel to unclassified and remove it from the volume
            lut.getRawData()[getIndex(lut, s[0], s[1], s[2])] = Colour::UNCLASSIFIED;
            --vol;
        }

        // Remove all non SA points from our new SA
        for(auto it = std::begin(newSA); it != std::end(newSA);) {
            auto& p = *it;

            // Remove if it's not the correct colour (it was also an SA voxel and was removed)
            if(getAt(lut, p[0], p[1], p[2]) != c) {
                it = newSA.erase(it);
            }

            // Remove if it's not an SA voxel now
            else if(!isSA(lut, p[0], p[1], p[2])) {
                it = newSA.erase(it);
            }

            // Otherwise move on
            else {
                ++it;
            }
        }

        // Store our new surface area
        sa = std::move(newSA);
    }

    LayerAutoClassifier::LayerAutoClassifier(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Trigger<Configuration<LayerAutoClassifier>>>([this](const Configuration<LayerAutoClassifier>& config) {

            //Loop through each classification char
            for(auto& c : config["limits"]) {
                maxVolume[static_cast<Colour>(c.first.as<char>())] = c.second["max_volume"].as<int>();
            }
        });

        // When we get a look up table then it was uploaded or emitted by someone else
        // We need to set it up for our datastructure
        on<Trigger<LookUpTable>>([this] (const LookUpTable& lut) {

            std::map<Colour, std::set<std::array<int, 3>>> newSA;
            std::map<Colour, int> newVol;

            for(int x = 0; x < (1 << lut.BITS_Y); ++x) {
                for (int y = 0; y < (1 << lut.BITS_CB); ++y) {
                    for (int z = 0; z < (1 << lut.BITS_CR); ++z) {

                        // Get our classification
                        Colour us = lut.getRawData()[(((x << lut.BITS_Y) | y) << lut.BITS_CB) | z];

                        // Ignore unclassified pixels
                        if(us != Colour::UNCLASSIFIED) {

                            // Increase our volume
                            ++newVol[us];

                            // If this is a surface area pixel add it to the list
                            if(isSA(lut, x, y, z)) {
                                newSA[us].insert({x, y, z});
                            }
                        }
                    }
                }
            }

            surfaceArea = std::move(newSA);
            volume = std::move(newVol);

            for(auto& v : volume) {
                log("Volume:", char(v.first), v.second);
            }

            for(auto& p : surfaceArea) {
                for(auto& i : p.second) {
                    log("SA:", char(p.first), i[0], i[1], i[2]);
                }
            }
        });

        on<Trigger<AutoClassifierPixels>, With<LookUpTable>>([this] (const AutoClassifierPixels& pixels, const LookUpTable& lut) {

            // Some aliases
            const auto& c = pixels.classification;
            auto& vol     = volume[c];
            auto& maxVol  = maxVolume[c];
            auto& sa      = surfaceArea[c];

            // Build up our differences in the look up table
            auto tableDiff = std::make_unique<LookUpTableDiff>();

            for(auto& pixel : pixels.pixels) {

                // Lookup the pixel
                auto colour = lut(pixel);

                // If it's an unclassified pixel then we can do something
                if(colour == Colour::UNCLASSIFIED) {

                    // Get our voxel coordinates for this pixel
                    int x = pixel.y  >> (8 - lut.BITS_Y);
                    int y = pixel.cb >> (8 - lut.BITS_CB);
                    int z = pixel.cr >> (8 - lut.BITS_CR);

                    // Get our surrounding voxels
                    auto surrounds = getSurrounding(lut, x, y, z);

                    // Check if we are touching a filled voxel of our colour
                    bool touching = std::find(std::begin(surrounds), std::end(surrounds), c) != std::end(surrounds);

                    if(touching) {
                        // We need a mutable lut
                        LookUpTable& mLut = *const_cast<LookUpTable*>(&lut);

                        // If we have exceeded max volume then shed
                        if(vol >= maxVol) {
                            // Emit the diff of the voxels we are about to remove
                            for(auto& s : sa) {
                                // Add our diff for displaying
                                auto& diff = *tableDiff->add_diff();
                                diff.set_lut_index(getIndex(lut, s[0], s[1], s[2]));
                                diff.set_classification(Colour::UNCLASSIFIED);
                            }

                            shed(mLut, c, sa, vol);
                        }
                        // Otherwise we proceed
                        else {
                            // Classify this new point
                            mLut.getRawData()[getIndex(lut, x, y, z)] = c;

                            // Our volume increases
                            ++vol;

                            // If we are an SA pixel add us
                            if(isSA(lut, x, y, z)) sa.insert({ x, y, z });

                            // Check if we made any of our neighbours internal
                            if(surrounds[0] == c && !isSA(lut, x + 1, y, z) && sa.find({ x + 1, y, z }) != std::end(sa)) sa.erase(sa.find({ x + 1, y, z })); // Remove this from the SA list if it is there
                            if(surrounds[1] == c && !isSA(lut, x - 1, y, z) && sa.find({ x - 1, y, z }) != std::end(sa)) sa.erase(sa.find({ x - 1, y, z })); // Remove this from the SA list if it is there
                            if(surrounds[2] == c && !isSA(lut, x, y + 1, z) && sa.find({ x, y + 1, z }) != std::end(sa)) sa.erase(sa.find({ x, y + 1, z })); // Remove this from the SA list if it is there
                            if(surrounds[3] == c && !isSA(lut, x, y - 1, z) && sa.find({ x, y - 1, z }) != std::end(sa)) sa.erase(sa.find({ x, y - 1, z })); // Remove this from the SA list if it is there
                            if(surrounds[4] == c && !isSA(lut, x, y, z + 1) && sa.find({ x, y, z + 1 }) != std::end(sa)) sa.erase(sa.find({ x, y, z + 1 })); // Remove this from the SA list if it is there
                            if(surrounds[5] == c && !isSA(lut, x, y, z - 1) && sa.find({ x, y, z - 1 }) != std::end(sa)) sa.erase(sa.find({ x, y, z - 1 })); // Remove this from the SA list if it is there

                            // Add our diff for displaying
                            auto& diff = *tableDiff->add_diff();
                            diff.set_lut_index(lut.getLUTIndex(pixel));
                            diff.set_classification(c);
                        }
                    }
                }
            }

            if (tableDiff->diff_size() > 0) {
                emit(std::move(tableDiff));
            }
        });
    }

}
}

