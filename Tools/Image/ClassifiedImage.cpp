#include "ClassifiedImage.h"

ClassifiedImage::ClassifiedImage(): imageWidth(0), imageHeight(0), internalBuffer(false)
{
    image = 0;
}

ClassifiedImage::ClassifiedImage(int width, int height, bool useInternalBuffer): imageWidth(width), imageHeight(height), internalBuffer(useInternalBuffer)
{
    image = 0;
    if(internalBuffer)
    {
        addInternalBuffer(width, height);
    }
}

ClassifiedImage::~ClassifiedImage()
{
    if (internalBuffer)
    {
        removeInternalBuffer();
    }
    delete [] image;
    image = 0;
}

void ClassifiedImage::useInternalBuffer(bool newCondition)
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

void ClassifiedImage::removeInternalBuffer()
{
    if (internalBuffer)
    {
        delete [] localBuffer;  // Delete the old buffer.
        delete [] image;        // Delete the array of line pointers.
        image = 0;
    }
    internalBuffer = false;
}

void ClassifiedImage::addInternalBuffer(int width, int height)
{
    unsigned char* buffer = allocateBuffer(width, height);
    MapBufferToImage(buffer, width, height);
    internalBuffer = true;
}

unsigned char* ClassifiedImage::allocateBuffer(int width, int height)
{
    int arrayWidth = width;
    unsigned char* buffer = new unsigned char[arrayWidth * height];
    localBuffer = buffer;
    return buffer;
}

void ClassifiedImage::MapBufferToImage(unsigned char* buffer, int width, int height)
{
    int arrayWidth = width;
    setImageDimensions(width, height);
    if(image == 0)
    {
        //Allocate memory for array of elements of column
        image = new unsigned char*[height];
    }
    // Now point the pointers in the right place
    int pixelIndex = 0;
    for( int i = 0; i < height; ++i)
    {
        image[i] = &buffer[pixelIndex];
        pixelIndex += arrayWidth;
    }
}

void ClassifiedImage::setImageDimensions(int newWidth, int newHeight)
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
