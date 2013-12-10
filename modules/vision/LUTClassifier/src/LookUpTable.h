/**
*       @name LookUpTable
*       @file lookuptable.h
*       @brief Wraps LUT buffer with access methods for pixel classification
*       @author Shannon Fenn
*       @date 17-02-12
*       @note ported by Jake Fountain Dec 2013 to NUClear system
*/

#ifndef LOOKUPTABLE_H
#define LOOKUPTABLE_H

#include "messages/input/Image.h"
#include "messages/vision/ClassifiedImage.h"
#include <nuclear>
#include <string>
#include <iostream>
#include <fstream>


namespace modules{
  namespace vision{
    
    class LookUpTable
    {
    public:
        static const int LUT_SIZE = 128*128*128; //!< The size of a lookup table in bytes.

        LookUpTable();
        LookUpTable(unsigned char* vals);

        /*!
            @brief sets a LUT given an array of values
            @param vals the array of values.
          */
        void set(unsigned char* vals);

        /*!
            @brief Loads a new LUT from a given file.
            @param filename The filename std::string.
            @return Returns the success of the operation.
        */
        bool loadLUTFromFile(const std::string& fileName);

        /*!
        *  @brief Classifies an individual pixel.
        *  @param p The pixel to be classified.
        *  @return Returns the classfied colour index for the given pixel.
        */
        void zero();

        const messages::vision::ClassifiedImage::Colour classifyPixel(const messages::input::Image::Pixel& p) const {
//            return getColourFromIndex(LUT[getLUTIndex(p)]); // 7bit LUT
            return (messages::vision::ClassifiedImage::Colour)(LUT[getLUTIndex(p)]); // 7bit LUT
        }        
        
    private:
        /*!
        *  @brief Gets the index of the pixel in the LUT
        *  @param p The pixel to be classified.
        *  @return Returns the colour index for the given pixel.
        */
        const unsigned int getLUTIndex(const messages::input::Image::Pixel& colour) const;
   
        const unsigned char* LUT;           //! @variable Colour Look Up Table - protected.
        unsigned char* LUTbuffer;           //! @variable temp LUT for loading.
    };
  } //vision
} // modules



#endif // LOOKUPTABLE_H
