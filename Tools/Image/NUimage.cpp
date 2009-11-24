#include "NUimage.h"

/*!
@file NUimage.h
@brief Declaration of NUbots NUimage class. Storage class for images.
*/

NUimage::NUimage(): imageWidth(0), imageHeight(0), internalBuffer(false)
{
    image = 0;
}

NUimage::NUimage(int width, int height, bool useInternalBuffer): imageWidth(width), imageHeight(height), internalBuffer(useInternalBuffer)
{
    image = 0;
    if(internalBuffer)
    {
        addInternalBuffer(width, height);
    }
}

NUimage::~NUimage()
{
    if (internalBuffer)
    {
        removeInternalBuffer();
    }
    delete [] image;
    image = 0;
}

void NUimage::useInternalBuffer(bool newCondition)
{
    if(internalBuffer == newCondition) return;
    if (internalBuffer == true)
    {
        // Remove old buffer.
        removeInternalBuffer();
    }
    else
    {
        addInternalBuffer(width(), height());
    }
}

void NUimage::removeInternalBuffer()
{
    if (internalBuffer)
    {
        delete [] *image;
        delete [] image;
        image = 0;
    }
    internalBuffer = false;
}

void NUimage::addInternalBuffer(int width, int height)
{
    pixels::Pixel* buffer = allocateBuffer(width, height);
    MapBufferToImage(buffer, width, height);
    internalBuffer = true;
}

pixels::Pixel* NUimage::allocateBuffer(int width, int height)
{
    int arrayWidth = 2*width;
    pixels::Pixel* buffer = new pixels::Pixel[arrayWidth * height];
    localBuffer = buffer;
    return buffer;
}

void NUimage::MapBufferToImage(pixels::Pixel* buffer, int width, int height)
{
    int arrayWidth = 2*width;
    if(height != this->height())
    {
        delete [] image;
        image = 0;
    }
    if(image == 0)
    {
        //Allocate memory for array of elements of column
        image = new pixels::Pixel*[height];
    }
    // Now point the pointers in the right place
    int pixelIndex = 0;
    for( int i = 0; i < height; ++i)
    {
        image[i] = &buffer[pixelIndex];
        pixelIndex += arrayWidth;
    }
}

void NUimage::setImageDimensions(int newWidth, int newHeight)
{
    if((imageWidth == newWidth) && (imageHeight == newHeight))
    {
        return;
    }
    if(internalBuffer == true)
    {
        removeInternalBuffer();
        addInternalBuffer(newWidth, newHeight);
    }
    imageWidth = newWidth;
    imageHeight = newHeight;
    return;
}
