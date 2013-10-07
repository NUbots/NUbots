/*
 * This file is part of AubioBeatDetector.
 *
 * AudioInput is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * AubioBeatDetector is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with AubioBeatDetector.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 Joshua Kearns <joshau-k@hotmail.com>
 */

#ifndef MODULES_AUBIOBEATDETECTOR_H
#define	MODULES_AUBIOBEATDETECTOR_H

#include <aubio/aubio.h>
#include <NUClear.h>


namespace modules {

    class AubioBeatDetector : public NUClear::Reactor {
    private:
        
        //Chunk resizing variables
        std::vector<int16_t> chunkWindow;
        
        //Beat Detection Trigger Variables
        aubio_tempo_t * att;// =  new_aubio_tempo (aubio_onset_kl, WINDOW_SIZE, HOP_SIZE, NUM_CHANNELS); //aubio_onset_kl or aubio_onset_complex (may be others too)
        fvec_t * out; //= new_fvec(HOP_SIZE , NUM_CHANNELS);
        fvec_t * tempo_input_data; //= new_fvec(WINDOW_SIZE , NUM_CHANNELS);
        int beatCount;
        long callCount;
        int sampleRate;
        int channels;
        int chunkSize;
        bool allowBeat;
    public:
        explicit AubioBeatDetector(NUClear::PowerPlant* plant);
        ~AubioBeatDetector();
    };
}

#endif	/* AUBIOBEATDETECTOR_H */

