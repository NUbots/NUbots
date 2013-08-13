/*!
  @file NUImage.cpp
  @brief Implementation of NUbots NUImage class.
  @author Steven Nicklin

  Copyright (c) 2009 Jason Kulk

 This file is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This file is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with NUbot.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "NUImage.h"
#include <cstring>

namespace messages {

    NUImage::NUImage(): image(nullptr), imageWidth(0), imageHeight(0), usingInternalBuffer(false), flipped(false) {
    }

    NUImage::NUImage(int width, int height, bool useInternalBuffer): image(nullptr), imageWidth(width), imageHeight(height), usingInternalBuffer(useInternalBuffer), flipped(false) {
        if(usingInternalBuffer) {
            addInternalBuffer(width, height);
        }
    }

    NUImage::NUImage(const NUImage& source): image(nullptr), imageWidth(0), imageHeight(0), usingInternalBuffer(false) {
        int sourceWidth = source.getWidth();
        int sourceHeight = source.getHeight();
        setImageDimensions(sourceWidth, sourceHeight);
        useInternalBuffer(true);
        timestamp = source.timestamp;
        for(int y = 0; y < sourceHeight; y++) {
            std::memcpy ( &image[y][0], &source.image[y][0], sizeof(source.image[y][0])*sourceWidth);
        }
        flipped = source.flipped;
    }

    NUImage::~NUImage() {
        if (usingInternalBuffer) {
            removeInternalBuffer();
        } else {
            delete [] image;
        }
    }

    void NUImage::copyFromExisting(const NUImage& source) {
        int sourceWidth = source.getWidth();
        int sourceHeight = source.getHeight();
        setImageDimensions(sourceWidth, sourceHeight);
        useInternalBuffer(true);

        for(int y = 0; y < sourceHeight; y++) {
            std::memcpy ( &image[y][0], &source.image[y][0], sizeof(source.image[y][0])*sourceWidth);
        }
        timestamp = source.timestamp;
        flipped = source.flipped;
    }

    void NUImage::cloneExisting(const NUImage& source) {
        int sourceWidth = source.getWidth();
        int sourceHeight = source.getHeight();
        bool sourceBuffered = source.getLocallyBuffered();
        if(sourceBuffered) {
            copyFromExisting(source);
        } else {
            mapYUV422BufferToImage((unsigned char*)&source.image[0][0], sourceWidth*2, sourceHeight*2, source.flipped);
        }
        timestamp = source.timestamp;
    }

    void NUImage::useInternalBuffer(bool newCondition) {
        if(usingInternalBuffer == newCondition) return;
        if (newCondition == true) {
            addInternalBuffer(getWidth(), getHeight());
        } else {
            // Remove old buffer.
            removeInternalBuffer();
        }
    }

    void NUImage::removeInternalBuffer() {
        if (usingInternalBuffer) {
            delete [] image;
            delete [] localBuffer;
            image = nullptr;
            localBuffer = nullptr;
        }
        usingInternalBuffer = false;
    }

    void NUImage::addInternalBuffer(int width, int height) {
        Pixel* buffer = allocateBuffer(width, height);
        mapBufferToImage(buffer, width, height);
        usingInternalBuffer = true;
    }

    Pixel* NUImage::allocateBuffer(int width, int height) {
        Pixel* buffer = new Pixel[width * height];
        localBuffer = buffer;
        return buffer;
    }

    void NUImage::mapYUV422BufferToImage(const unsigned char* buffer, int width, int height, bool flip) {
        useInternalBuffer(false);
        int arrayWidth = width;
        // halve the width and height since we want to skip
        width /= 2;
        height /= 2;
        Pixel* pixelisedBuffer = (Pixel*) buffer;
        if(height != this->getHeight()) {
            delete [] image;
            image = 0;
        }
        if(image == 0) {
            //Allocate memory for array of elements of column
            image = new Pixel*[height];
        }
        // Now point the pointers in the right place
        int pixelIndex = 0;
        for( int i = 0; i < height; ++i) {
            image[i] = &pixelisedBuffer[pixelIndex];
            pixelIndex += arrayWidth;
        }
        imageWidth = width;
        imageHeight = height;
        flipped = flip;
    }

    void NUImage::copyFromYUV422Buffer(const unsigned char* buffer, int width, int height, bool flip) {
        width /= 2;
        height /= 2;
        setImageDimensions(width, height);
        useInternalBuffer(true);
        Pixel* pixelisedBuffer = (Pixel*)buffer;
        for(int y = 0; y < height; y++) {
           for(int x = 0; x < width; x++) {
               this->image[y][x] = pixelisedBuffer[y*width*2 + x];
           }
        }
        flipped = flip;
    }

    void NUImage::mapBufferToImage(Pixel* buffer, int width, int height, bool flip) {
        if(height != this->getHeight()) {
            delete [] image;
            image = nullptr;
        }
        if(image == 0) {
            //Allocate memory for array of elements of column
            image = new Pixel*[height];
        }
        // Now point the pointers in the right place
        int pixelIndex = 0;
        for( int i = 0; i < height; ++i) {
            image[i] = &buffer[pixelIndex];
            pixelIndex += width;
        }
        imageWidth = width;
        imageHeight = height;
        flipped = flip;
    }

    void NUImage::setImageDimensions(int newWidth, int newHeight) {
        // If the image size has not changed do nothing.
        if((imageWidth == newWidth) && (imageHeight == newHeight)) {
            return;
        }

        if(usingInternalBuffer == true) {
            removeInternalBuffer();
            addInternalBuffer(newWidth, newHeight);
        }
        imageWidth = newWidth;
        imageHeight = newHeight;
    }


    /*! @brief Put the entire contents of the NUSensorsData class into a stream
     */
    std::ostream& operator<< (std::ostream& output, const NUImage& p_image) {
        NUImage::Header image_header = NUImage::currentVersionHeader();
        int sourceWidth = p_image.getWidth();
        int sourceHeight = p_image.getHeight();
        double timeStamp = p_image.timestamp;
        bool flipped = p_image.flipped;
        output.write(reinterpret_cast<char*>(&image_header), sizeof(image_header));
        output.write(reinterpret_cast<char*>(&sourceWidth), sizeof(sourceWidth));
        output.write(reinterpret_cast<char*>(&sourceHeight), sizeof(sourceHeight));
        output.write(reinterpret_cast<char*>(&timeStamp), sizeof(timeStamp));
        output.write(reinterpret_cast<char*>(&flipped), sizeof(flipped));
        for(int y = 0; y < sourceHeight; y++) {
            output.write(reinterpret_cast<char*>(p_image.image[y]), sizeof(p_image.image[y][0])*sourceWidth);
        }
        return output;
    }

    std::istream& operator>> (std::istream& input, NUImage& p_image) {
        int width, height;
        //std::streampos start = input.tellg();
        NUImage::Header header;
        NUImage::Version image_version = NUImage::VERSION_UNKNOWN;
        input.read(reinterpret_cast<char*>(&header), sizeof(header));
        if(NUImage::validHeader(header) == true) {
            image_version = header.version;
        } else {
            throw std::exception();
            //input.seekg(start);
        }

        input.read(reinterpret_cast<char*>(&width), sizeof(width));
        input.read(reinterpret_cast<char*>(&height), sizeof(height));
        input.read(reinterpret_cast<char*>(&p_image.timestamp), sizeof(p_image.timestamp));
        if(image_version >= NUImage::VERSION_001) {
            input.read(reinterpret_cast<char*>(&p_image.flipped), sizeof(p_image.flipped));
        }
        p_image.setImageDimensions(width, height);
        p_image.useInternalBuffer(true);
        for(int y = 0; y < height; y++) {
            if(!input.good()) throw std::exception();
            input.read((char*) p_image.image[y], sizeof(p_image.image[y][0])*width);
        }
        return input;
    }

}
