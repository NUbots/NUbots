/*!
  @file NUImage.h 
  @brief Declaration of NUbots NUImage class.
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

#ifndef MESSAGES_NUIMAGE_H
#define MESSAGES_NUIMAGE_H

#include "CameraSettings.h"
#include "Pixel.h"
#include <iostream>

namespace messages {
/*!
@brief Class used to store an image and its relevant information.

This is the standard image format that images from different platforms must be
converted to so that they can be used with the NUbot code base. By converting images 
to this standard image format we are able to have our software support multiple robot 
platforms.
*/

#define NUIMAGE_HEADER_IDENTIFIER_LENGTH 4
#define NUIMAGE_IDENTIFIER "IMAG"

    class NUImage {
    public:
        enum Version {
            VERSION_UNKNOWN,
            VERSION_001,
            VERSIONS_TOTAL
        };

        struct Header {
            char identifier[NUIMAGE_HEADER_IDENTIFIER_LENGTH];
            Version version;
        };

        static Header currentVersionHeader() {
            Header currHeader;
            const Version currVersion = VERSION_001;
            for(unsigned int i = 0; i < NUIMAGE_HEADER_IDENTIFIER_LENGTH ; ++i) {
                currHeader.identifier[i] = NUIMAGE_IDENTIFIER[i];
            }
            currHeader.version = currVersion;
            return currHeader;
        }

        static bool validHeader(const Header& header) {
            for(unsigned int i = 0; i <  NUIMAGE_HEADER_IDENTIFIER_LENGTH ; ++i) {
                if(header.identifier[i] != NUIMAGE_IDENTIFIER[i]) {
                    return false;
                }
            }
            return true;
        }

        /*!
        @brief Default constructor.
        */
        NUImage();

        /*!
        @brief Constructor with initial settings.
        @param width Intial image width.
        @param height Intial image height.
        @param useInternalBuffer Initial buffering setting.
        True and internal buffer is created. False it is not.
        */
        NUImage(int width, int height, bool useInternalBuffer);

        /*!
        @brief Copy constructor. Creates a deep copy of the source image.
        @param source The image from which to base the new class.
        */
        NUImage(const NUImage& source);

        /*!
        @brief Destructor.
        */
        ~NUImage();

        /*!
        @brief Creates a local copy of the source image.
        If the source image references an external buffer, a local copy of this buffer will be made.
        @param source The source image.
        */
        void copyFromExisting(const NUImage& source);

        /*!
        @brief Creates an exact copy of the source image.
        If the source image references an external buffer, the clone will reference this same buffer.
        @param source The source image.
        */
        void cloneExisting(const NUImage& source);

        /*!
        @brief Maps a one-dimensional buffer to the two-dimensional image array.
        @param buffer The one-dimensional buffer.
        @param width The width of the image array.
        @param height The height of the image array.
        */
        void mapBufferToImage(Pixel* buffer, int width, int height, bool flip = false);

        /*!
        @brief Maps a YUV422 formatted image buffer to the image. A local copy IS NOT made.
        @param buffer The YUV422 image.
        @param width The width of the image.
        @param height The height of the image.
        */
        void mapYUV422BufferToImage(const unsigned char* buffer, int width, int height, bool flip = false);

        /*!
        @brief Copies a YUV422 formatted image buffer to the image. A local copy IS made.
        @param buffer The YUV422 image.
        @param width The width of the image.
        @param height The height of the image.
        */
        void copyFromYUV422Buffer(const unsigned char* buffer, int width, int height, bool flip = false);

        /*!
        @brief Output streaming operation.
        @param output The output stream.
        @param p_nuimage The source image to be streamed.
        */
        friend std::ostream& operator<< (std::ostream& output, const NUImage& p_nuimage);

        /*!
        @brief Input streaming operation.
        @param input The input stream.
        @param p_nuimage The destination image to be streamed to.
        */
        friend std::istream& operator>> (std::istream& input, NUImage& p_nuimage);

        /*!
        @brief Get the width of the current image.
        @return The image width.
        */
        int getWidth() const {
            return imageWidth;
        }
    
        /*!
        @brief Get the height of the current image.
        @return The image height.
        */
        int getHeight() const {
            return imageHeight;
        }

        /*!
        @brief Gets whether the image is flipped in memory.
        @return Whether the image is flipped in memory.
        */
        bool getFlipped() const {
            return flipped;
        }

        /*!
        @brief Get the total number of pixels within the current image.
        @return The number of pixels in the image.
        */
        int getTotalPixels() const {
            return getWidth() * getHeight();
        }

        void setPixel(unsigned int x, unsigned int y, Pixel px) {
            if(flipped) {
                image[getHeight() - y - 1][getWidth() - x - 1] = px;
            } else {
                image[y][x] = px;
            }
        }

        /*!
        @brief Image access operator, access the pixel at the desired position of the image.

        This access operator allows the image coordinates to be mapped to different buffer positions.

        @param x Image x coordinate
        @param y Image y coordinate
        @return The pixel at the coordinate (x,y)
        */
        Pixel operator()(unsigned int x, unsigned int y) const {
            if(flipped) {
                return at(getWidth() - x - 1, getHeight() - y - 1);
            } else {
                return at(x, y);
            }
        }

        /*!
        @brief Image raw access operator, access the pixel at the desired image buffer position.

        This access operator allows direct access to the image buffer.

        @param x Buffer x position
        @param y Buffer y position
        @return The pixel at the buffer position (x,y)
        */
        Pixel at(unsigned int x, unsigned int y) const {
            return image[y][x];
        }

        /*!
        @brief Get the current buffering state of the image.
        @return The current buffering state. True when buffered internally. False when buffered externally.
        */
        bool getLocallyBuffered() const {
            return usingInternalBuffer;
        }

        /*!
        @brief Gets the time at which the image was generated.
        @return The image's timestamp.
        */
        double getTimestamp() const {
            return timestamp;
        }
    
        /*!
        @brief Gets the camera settings that were used when the image was taken
        @return The image's camera settings.
        */
        const CameraSettings& getCameraSettings() const {
            return currentCameraSettings;
        }
    
        void setCameraSettings(const CameraSettings& newCameraSettings) {
            currentCameraSettings = newCameraSettings;
        }

        void setTimestamp(double time) {
            timestamp = time;
        }

    private:
        Pixel **image;                        //!< Pointer to the image array.
        double timestamp;		              //!< Time point at which the image was captured. (Unix Time)
        int imageWidth;                       //!< The current image width.
        int imageHeight;                      //!< The current image height.
        bool usingInternalBuffer;             //!< The current image buffering state. True when buffered internally. false when buffered externally.
        Pixel *localBuffer;                   //!< Pointer to the local storage buffer.
        bool flipped;                         //!< Whether the image is flipped in memory.
        CameraSettings currentCameraSettings; //!< Copy Of Current Camera Settings.
        /*!
        @brief Selects the buffering mode for the image.
        @param newCondition Select the new buffering mode. True the image is buffered internally.
        False it is not.
        */
        void useInternalBuffer(bool newCondition = true);

        /*!
        @brief Change the image dimensions.
        @param newWidth The new width of the image.
        @param newHeight The new height of the image.
        */
        void setImageDimensions(int newWidth, int newHeight);

        /*!
        @brief Removes the current internal buffer.
        */
        void removeInternalBuffer();

        /*!
        @brief Adds a new internal buffer of the specified dimensions.
        @param width The width of the new internal buffer.
        @param height The height of the new internal buffer.
        */
        void addInternalBuffer(int width, int height);

        /*!
        @brief Create a buffer of the given dimensions.
        @param width The width of the new buffer.
        @param height The height of the new buffer.
        @return The new buffer.
        */
        Pixel* allocateBuffer(int width, int height);
    };

}
#endif
