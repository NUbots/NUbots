/*!
  @file Image.h 
  @brief Declaration of an image class.
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

#ifndef MESSAGES_IMAGE_H
#define MESSAGES_IMAGE_H

#include <memory>

namespace messages {
/*!
@brief Class used to store an image and its relevant information.
*/

    class Image {
    public:
        union Pixel {
            unsigned int color;         //!< Representation as single machine word.
            unsigned char channel[4];   //!< Representation as an array of channels.
            struct {
                unsigned char yCbCrPadding, //!< Ignore
                              cb,           //!< Cb channel.
                              y,            //!< Y channel.
                              cr;           //!< Cr channel.
            };
        };

        /*!
        @brief Default constructor.
        */
        Image() : imageWidth(0), imageHeight(0) {}

        /*!
        @brief Constructor with initial settings.
        @param width Intial image width.
        @param height Intial image height.
        True and internal buffer is created. False it is not.
        */
        Image(std::size_t width, std::size_t height) : image(new Pixel[width * height]), imageWidth(width), imageHeight(height) {}

        /*!
        @brief Creates an image from a buffer of pixels in YUV 4:2:2 format
        @param buffer The buffer to copy from
        @param width The width of the image in the buffer
        @param height The height of the image in the buffer
        @param flip Whether the buffer is in reverse order (right-to-left, bottom-to-top)
        */
        void copyFromYUV422Buffer(const Pixel* buffer, std::size_t width, std::size_t height, bool flip = false);

        /*!
        @brief Get the width of the current image.
        @return The image width.
        */
        std::size_t getWidth() const {
            return imageWidth;
        }
    
        /*!
        @brief Get the height of the current image.
        @return The image height.
        */
        std::size_t getHeight() const {
            return imageHeight;
        }

        /*!
        @brief Get the total number of pixels within the current image.
        @return The number of pixels in the image.
        */
        std::size_t getTotalPixels() const {
            return getWidth() * getHeight();
        }

        /*!
        @brief Image access operator, access the pixel at the desired position of the image.

        This access operator allows the image coordinates to be mapped to different buffer positions.

        @param x Image x coordinate
        @param y Image y coordinate
        @return The pixel at the coordinate (x,y)
        */
        Pixel& operator()(std::size_t x, std::size_t y) {
            return image[y * imageWidth + x];
        }

        /*!
        @brief Image access operator, access the pixel at the desired position of the image.

        This access operator allows the image coordinates to be mapped to different buffer positions.

        @param x Image x coordinate
        @param y Image y coordinate
        @return The pixel at the coordinate (x,y)
        */
        const Pixel& operator()(std::size_t x, std::size_t y) const {
            return image[y * imageWidth + x];
        }

    private:
        std::unique_ptr<Pixel[]> image;    //!< Pointer to the image array.
        std::size_t imageWidth;            //!< The current image width.
        std::size_t imageHeight;           //!< The current image height.
    };

}
#endif
