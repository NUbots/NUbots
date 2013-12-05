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
#include <nuclear>
#include <string>
#include <iostream>
#include <fstream>


namespace modules{
  namespace vision{

    enum Colour{
        unclassified, //!< Colour has not be given a category.
        white, //!< Colour is in the White region.
        green, //!< Colour is in the Green region.
        shadow_object, //!< Colour is part of a shadowed area.
        pink, //!< Colour is in the Red region.
        pink_orange, //!< Colour is in the region of overlap between Red and Orange.
        orange, //!< Colour is in the Orange region.
        yellow_orange, //!< Colour is in the region of overlap between Yellow and Orange.
        yellow, //!< Colour is in the Yellow region.
        blue, //!< Colour is in the Sky Blue region.
        shadow_blue, //!< Colour is in the Dark Blue region.
        num_colours, //!< Total number of colour categories.
        invalid
    };  
    
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

        Colour classifyPixel(const messages::input::Image::Pixel& p)
        {
            return getColourFromIndex(LUT[getLUTIndex(p)]); // 7bit LUT
        }
    private:
        /*!
        *  @brief Gets the index of the pixel in the LUT
        *  @param p The pixel to be classified.
        *  @return Returns the colour index for the given pixel.
        */
        unsigned int getLUTIndex(const messages::input::Image::Pixel& colour);
        /*!
        *  @param p The Colour enum value of the pixel to be classified.
        *  @return Returns the classified colour for the given pixel colour number.
        */
        Colour getColourFromIndex(int index);        
        /*!
        Gets the name of the given colour.
        @param colour The colour name desired.
        @return The name of the colour.
        */
        static std::string getColourName(const Colour& colour);

   
        const unsigned char* LUT;           //! @variable Colour Look Up Table - protected.
        unsigned char* LUTbuffer;           //! @variable temp LUT for loading.
    };
  } //vision
} // modules



#endif // LOOKUPTABLE_H
