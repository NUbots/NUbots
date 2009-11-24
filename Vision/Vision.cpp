/*!
  @file Vision.h
  @brief Declaration of NUbots Vision class.
  @author Steven Nicklin
*/
#include "Vision.h"
#include "Tools/Image/NUImage.h"


Vision::Vision()
{
    return;
}

Vision::~Vision()
{
    return;
}

unsigned char Vision::classifyPixel(int x, int y)
{
    pixels::Pixel* temp = &currentImage->image[y][x];
    return currentLookupTable[(temp->y<<16) + (temp->cb<<8) + temp->cr];
}

void Vision::classifyImage(ClassifiedImage &target, const NUimage* sourceImage, const unsigned char *lookUpTable)
{   
    target.setImageDimensions(sourceImage->width(),sourceImage->height());
    currentImage = sourceImage;
    currentLookupTable = lookUpTable;
    for (int y = 0; y < sourceImage->height(); y++)
    {
        for (int x = 0; x < sourceImage->width(); x++)
        {
            target.image[y][x] = classifyPixel(x,y);
        }
    }
    return;
}
