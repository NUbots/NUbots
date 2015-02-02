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


    inline int getIndex(const LookUpTable& lut, const uint8_t& x, const uint8_t& y, const uint8_t& z) {
        return (((x << lut.BITS_CR) | y) << lut.BITS_CB) | z;
    }

    inline int getIndex(const LookUpTable& lut, const Image::Pixel& p) {
        int x = p.y  >> (8 - lut.BITS_Y);
        int y = p.cb >> (8 - lut.BITS_CB);
        int z = p.cr >> (8 - lut.BITS_CR);

        return getIndex(lut, x, y, z);
    }

    inline const Colour& getAt(const LookUpTable& lut, const int& index) {
        return lut.getRawData()[index];
    }

    inline Colour& getAt(LookUpTable& lut, const int& index) {
        return lut.getRawData()[index];
    }

    inline bool isTouching(const LookUpTable& lut, const int& index, const Colour& c) {

        // Loop through each axis and check any are filled
        for(int i : { 1, 1 << lut.BITS_CB, 1 << (lut.BITS_Y + lut.BITS_CB) }) {
            if(getAt(lut, index + i) == c
            || getAt(lut, index - i) == c) {
                return true;
            }
        }
        return false;
    }

    inline bool isInternal(const LookUpTable& lut, const int& index) {

        // Get the classification for this pixel
        Colour c = getAt(lut, index);

        // Will be true if this voxel is internal
        bool internal = true;

        // Loop through each axis and check they are all filled
        for(int i : { 1, 1 << lut.BITS_CB, 1 << (lut.BITS_Y + lut.BITS_CB) }) {
            internal &= getAt(lut, index + i) == c;
            internal &= getAt(lut, index - i) == c;
        }

        return internal;
    }

    inline bool isRemoveable(const LookUpTable& lut, const int& index) {

        // Get the classification for this pixel
        Colour c = getAt(lut, index);

        // Internal is never removeable
        if(isInternal(lut, index)) {
            return false;
        }

        // Will be true if this voxel is removeable
        bool removeable = false;

        // Loop through each axis
        for(int i : { 1, 1 << lut.BITS_CB, 1 << (lut.BITS_Y + lut.BITS_CB) }) {

            // Check if we have a filled cell opposite to a non filled cell
            if(!(getAt(lut, index + i) == c && getAt(lut, index - i) == c)) {

                // Check if we have a 3 long segment (only) in this axis
                if((getAt(lut, index - (2 * i)) == c && getAt(lut, index + i != c))
                || (getAt(lut, index + (2 * i)) == c && getAt(lut, index - i != c))) {
                    // This could be a surface removeable (provided the else doesn't fire)
                    removeable = true;
                }
            }
            else {
                // Break! this isn't surface by this criteria!
                removeable = false;
                break;
            }
        }

        // If we are not removeable yet
        if(!removeable) {

            // The number of internal voxels
            int internal = 0;
            // The number of voxels that are internal and not opposite another internal
            int nonOppositeInternal = 0;

            // Loop through each axis again
            for(int i : { 1, 1 << lut.BITS_CB, 1 << (lut.BITS_Y + lut.BITS_CB) }) {

                // Check if either side is internal
                bool a = getAt(lut, index + i) == c && isInternal(lut, index + 1);
                bool b = getAt(lut, index - i) == c && isInternal(lut, index - 1);

                // Add to our internal counts
                internal += a + b;
                nonOppositeInternal += a != b;

                if(internal >= 3 || nonOppositeInternal >= 2) {
                    // We are now removeable
                    removeable = true;
                    break;
                }
            }
        }

        return removeable;
    }

    void shed(LookUpTable& lut, Colour c, std::set<int>& sa, int& vol) {
        // Holds our new surface voxels
        std::set<int> newSA;

        // Go through the surface area for this colour
        for(auto& s : sa) {

            // Loop through each axis and insert the points as they may be SA now
            for(int i : { 1, 1 << lut.BITS_CB, 1 << (lut.BITS_Y + lut.BITS_CB) }) {
                newSA.insert(s + i);
                newSA.insert(s - i);
            }

            // Set this voxel to unclassified and remove it from the volume
            getAt(lut, s) = Colour::UNCLASSIFIED;
            --vol;
        }

        // Remove all non SA points from our new SA
        for(auto it = std::begin(newSA); it != std::end(newSA);) {
            auto& p = *it;

            // Remove if it's not the correct colour (it was also an SA voxel and was removed)
            // Remove if it's not an SA voxel now
            if(getAt(lut, p) != c || !isRemoveable(lut, p)) {
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
            for(auto& limit : config["limits"]) {
                Colour c          = static_cast<Colour>(limit.first.as<char>());
                maxVolume[c]      = limit.second["max_volume"].as<int>();
                maxSurfaceArea[c] = limit.second["surface_area_volume_ratio"].as<double>() * maxVolume[c];
            }
        });

        // When we get a look up table then it was uploaded or emitted by someone else
        // We need to set it up for our datastructure
        on<Trigger<LookUpTable>>([this] (const LookUpTable& lut) {

            std::map<Colour, std::set<int>> newSA;
            std::map<Colour, int> newVol;

            for(int x = 0; x < (1 << lut.BITS_Y); ++x) {
                for (int y = 0; y < (1 << lut.BITS_CB); ++y) {
                    for (int z = 0; z < (1 << lut.BITS_CR); ++z) {

                        // Get our index and classification
                        int index = getIndex(lut, x, y, z);
                        Colour c = getAt(lut, index);

                        // Ignore unclassified pixels
                        if(c != Colour::UNCLASSIFIED) {

                            // Increase our volume
                            ++newVol[c];

                            // If this removeable surface then add it to our list
                            if(isRemoveable(lut, index)) {
                                newSA[c].insert(index);
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


            // For visualization we are flagging surface voxels
            auto tableDiff = std::make_unique<LookUpTableDiff>();

            for(auto& p : surfaceArea) {
                log("SA: ", char(p.first), p.second.size());
                for(auto& i : p.second) {
                    log("    ", char(p.first), i);


                    // Add our diff for displaying
                    auto& diff = *tableDiff->add_diff();
                    diff.set_lut_index(i);
                    diff.set_classification(Colour::UNCLASSIFIED);

                }
            }

            // emit(std::move(tableDiff));
        });

        on<Trigger<AutoClassifierPixels>, With<LookUpTable>, Options<Single>>([this] (const AutoClassifierPixels& pixels, const LookUpTable& lut) {

            // Some aliases
            const auto& c = pixels.classification;
            auto& vol     = volume[c];
            auto& maxVol  = maxVolume[c];
            auto& sa      = surfaceArea[c];
            auto& maxSA   = maxSurfaceArea[c];

            // Build up our differences in the look up table
            auto tableDiff = std::make_unique<LookUpTableDiff>();

            for(auto& p : pixels.pixels) {


                // Lookup the pixel
                auto colour = lut(p);

                // If it's an unclassified pixel then we can do something
                if(colour == Colour::UNCLASSIFIED) {

                    // Get our voxel coordinates for this pixel
                    int index = getIndex(lut, p);

                    // Check if we are touching a filled voxel
                    if(isTouching(lut, index, c)) {

                        // We are going to need a mutable lut
                        LookUpTable& mLut = *const_cast<LookUpTable*>(&lut);

                        // If we have exceeded max volume then shed
                        if(vol >= maxVol) {
                            // Emit the diff of the voxels we are about to remove
                            for(auto& s : sa) {
                                // Add our diff for displaying
                                auto& diff = *tableDiff->add_diff();
                                diff.set_lut_index(s);
                                diff.set_classification(Colour::UNCLASSIFIED);
                            }

                            // Shed our voxel layer
                            shed(mLut, c, sa, vol);
                        }
                        // Otherwise we can classify this
                        else {
                            // Classify
                            getAt(mLut, index) = c;

                            // Volume increase
                            ++vol;

                            // If the new voxel is SA add it to the SA list
                            if(isRemoveable(lut, index)) {
                                sa.insert(index);

                                // Loop through each axis and insert the points as they may be SA now
                                for(int i : { 1, 1 << lut.BITS_CB, 1 << (lut.BITS_Y + lut.BITS_CB) }) {

                                    // Add or remove as necessary
                                    if(isRemoveable(lut, index + i)) {
                                        sa.insert(index + i);
                                    }
                                    else if(sa.find(index + i) != std::end(sa)) {
                                        sa.erase(sa.find(index + i));
                                    }
                                    if(isRemoveable(lut, index - i)) {
                                        sa.insert(index - i);
                                    }
                                    else if(sa.find(index - i) != std::end(sa)) {

                                        sa.erase(sa.find(index - i));
                                    }
                                }

                                // TODO check if we can remove any from the list
                            }

                            // Add our diff for displaying
                            auto& diff = *tableDiff->add_diff();
                            diff.set_lut_index(index);
                            diff.set_classification(c);

                            // If we exceeded our SA constraint then shed
                            if(sa.size() >= maxSA) {
                                // Emit the diff of the voxels we are about to remove
                                for(auto& s : sa) {
                                    // Add our diff for displaying
                                    auto& diff = *tableDiff->add_diff();
                                    diff.set_lut_index(s);
                                    diff.set_classification(Colour::UNCLASSIFIED);
                                }

                                shed(mLut, c, sa, vol);
                            }
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

