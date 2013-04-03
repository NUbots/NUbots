/**
*       @name LookUpTable
*       @file lookuptable.h
*       @brief Wraps LUT buffer with access methods for pixel classification
*       @author Shannon Fenn
*       @date 17-02-12
*
*/

#ifndef LOOKUPTABLE_H
#define LOOKUPTABLE_H

#include <string>
#include "Tools/FileFormats/LUTTools.h"
#include "Vision/VisionTools/classificationcolours.h"
#include "Infrastructure/NUImage/NUImage.h"
#include "debug.h"

using namespace Vision;

class LookUpTable
{
public:
    LookUpTable();
    LookUpTable(unsigned char* vals);

    /*!
      @brief sets a LUT given an array of values
      @param vals the array of values.
      */
    void set(unsigned char* vals);

    /*!
      @brief Loads a new LUT from a given file.
      @param filename The filename string.
      @return Returns the success of the operation.
      */
    bool loadLUTFromFile(const string& fileName);

    /*!
    *  @brief Classifies an individual pixel.
    *  @param p The pixel to be classified.
    *  @return Returns the classfied colour index for the given pixel.
    */
    inline Colour classifyPixel(const Pixel& p) const
    {
        //return  currentLookupTable[(temp->y<<16) + (temp->cb<<8) + temp->cr]; //8 bit LUT
        return getColourFromIndex(LUT[LUTTools::getLUTIndex(p)]); // 7bit LUT
    }

//    /*!
//      @brief classifies and entire image into an opencv Mat
//      @param src the image to classify.
//      @param dest the resulting classified image.
//      */
//    void classifyImage(const NUImage& src, cv::Mat& dest) const;

    void zero();

private:
    const unsigned char* LUT;           //! @variable Colour Look Up Table - protected.
    unsigned char* LUTbuffer;           //! @variable temp LUT for loading.
};

#endif // LOOKUPTABLE_H
