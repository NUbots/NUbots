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
    pixels::Pixel* buffer = new pixels::Pixel[width * height];
    localBuffer = buffer;
    return buffer;
}

void NUimage::MapYUV422BufferToImage(pixels::Pixel* buffer, int width, int height)
{
    int arrayWidth = width;
    // halve the width and height since we want to skip
    width /= 2;
    height /= 2;
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
    imageWidth = width;
    imageHeight = height;
}

void NUimage::CopyFromYUV422Buffer(unsigned char* buffer, int width, int height)
{
    width /= 2;
    height /= 2;
    setImageDimensions(width, height);
    useInternalBuffer(true);
    pixels::Pixel* pixelisedBuffer = (pixels::Pixel*)buffer;
    for(int y = 0; y < height; y++)
    {
       for(int x = 0; x < width; x++)
       {
           this->image[y][x] = pixelisedBuffer[y*width*2 + x];
       }
    }
    return;
}

void NUimage::MapBufferToImage(pixels::Pixel* buffer, int width, int height)
{
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
        pixelIndex += width;
    }
    imageWidth = width;
    imageHeight = height;
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

/*! @brief Put the entire contents of the NUSensorsData class into a stream
 */
std::ostream& operator<< (std::ostream& output, const NUimage& p_image)
{
    output << p_image.imageWidth << " ";
    output << p_image.imageHeight << " ";

    for(int y = 0; y <p_image.imageHeight; y++)
    {
       for(int x = 0; x <p_image.imageWidth; x++)
        {
            output.write((char*) &p_image.image[y][x], sizeof(p_image.image[y][x]));
        }
    }
    return output;
}

/*! @brief Get the entire contents of the NUSensorsData class from a stream
 */

std::istream& operator>> (std::istream& input, NUimage& p_image)
{
    int width, height;
    char temp;
    input >> width;
    input >> height;
    p_image.useInternalBuffer();
    p_image.setImageDimensions(width, height);

    input.read(&temp, sizeof(char));         // skip over the single space after the size
    for(int y = 0; y < height; y++)
    {
       for(int x = 0; x < width; x++)
        {
            input.read((char*) &p_image.image[y][x], sizeof(p_image.image[y][x]));
        }
    }
    return input;
}
