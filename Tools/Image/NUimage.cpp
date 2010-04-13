#include "NUimage.h"
#include <cstring>
/*!
@file NUimage.h
@brief Declaration of NUbots NUimage class. Storage class for images.
*/

NUimage::NUimage(): m_imageWidth(0), m_imageHeight(0), m_usingInternalBuffer(false)
{
    m_image = 0;
}

NUimage::NUimage(int width, int height, bool useInternalBuffer): m_imageWidth(width), m_imageHeight(height), m_usingInternalBuffer(useInternalBuffer)
{
    m_image = 0;
    if(m_usingInternalBuffer)
    {
        addInternalBuffer(width, height);
    }
}

NUimage::NUimage(const NUimage& source): m_imageWidth(0), m_imageHeight(0), m_usingInternalBuffer(false)
{
    m_image = 0;
    int sourceWidth = source.getWidth();
    int sourceHeight = source.getHeight();
    setImageDimensions(sourceWidth, sourceHeight);
    useInternalBuffer(true);
    m_timestamp = source.m_timestamp;
    for(int y = 0; y < sourceHeight; y++)
    {
        memcpy ( &m_image[y][0], &source.m_image[y][0], sizeof(source.m_image[y][0])*sourceWidth);
    }
}

NUimage::~NUimage()
{
    if (m_usingInternalBuffer)
    {
        removeInternalBuffer();
    }
    else
    {
        delete [] m_image;
        m_image = 0;
    }
}

void NUimage::copyFromExisting(const NUimage& source)
{
    int sourceWidth = source.getWidth();
    int sourceHeight = source.getHeight();
    setImageDimensions(sourceWidth, sourceHeight);
    useInternalBuffer(true);

    for(int y = 0; y < sourceHeight; y++)
    {
        memcpy ( &m_image[y][0], &source.m_image[y][0], sizeof(source.m_image[y][0])*sourceWidth);
    }
    m_timestamp = source.m_timestamp;
}

void NUimage::cloneExisting(const NUimage& source)
{
    int sourceWidth = source.getWidth();
    int sourceHeight = source.getHeight();
    bool sourceBuffered = source.getLocallyBuffered();
    if(sourceBuffered)
    {
        copyFromExisting(source);
    }
    else
    {
        MapYUV422BufferToImage((unsigned char*)&source.m_image[0][0], sourceWidth, sourceHeight);
    }
    m_timestamp = source.m_timestamp;
}

void NUimage::useInternalBuffer(bool newCondition)
{
    if(m_usingInternalBuffer == newCondition) return;
    if (newCondition == true)
    {
        addInternalBuffer(getWidth(), getHeight());
    }
    else
    {
        // Remove old buffer.
        removeInternalBuffer();
    }
}

void NUimage::removeInternalBuffer()
{
    if (m_usingInternalBuffer)
    {
        delete [] m_image;
        delete [] m_localBuffer;
        m_image = 0;
        m_localBuffer = 0;
    }
    m_usingInternalBuffer = false;
}

void NUimage::addInternalBuffer(int width, int height)
{
    Pixel* buffer = allocateBuffer(width, height);
    MapBufferToImage(buffer, width, height);
    m_usingInternalBuffer = true;
}

Pixel* NUimage::allocateBuffer(int width, int height)
{
    Pixel* buffer = new Pixel[width * height];
    m_localBuffer = buffer;
    return buffer;
}

void NUimage::MapYUV422BufferToImage(const unsigned char* buffer, int width, int height)
{
    useInternalBuffer(false);
    int arrayWidth = width;
    // halve the width and height since we want to skip
    width /= 2;
    height /= 2;
    Pixel* pixelisedBuffer = (Pixel*) buffer;
    if(height != this->getHeight())
    {
        delete [] m_image;
        m_image = 0;
    }
    if(m_image == 0)
    {
        //Allocate memory for array of elements of column
        m_image = new Pixel*[height];
    }
    // Now point the pointers in the right place
    int pixelIndex = 0;
    for( int i = 0; i < height; ++i)
    {
        m_image[i] = &pixelisedBuffer[pixelIndex];
        pixelIndex += arrayWidth;
    }
    m_imageWidth = width;
    m_imageHeight = height;
}

void NUimage::CopyFromYUV422Buffer(const unsigned char* buffer, int width, int height)
{
    width /= 2;
    height /= 2;
    setImageDimensions(width, height);
    useInternalBuffer(true);
    Pixel* pixelisedBuffer = (Pixel*)buffer;
    for(int y = 0; y < height; y++)
    {
       for(int x = 0; x < width; x++)
       {
           this->m_image[y][x] = pixelisedBuffer[y*width*2 + x];
       }
    }
    return;
}

void NUimage::MapBufferToImage(Pixel* buffer, int width, int height)
{
    if(height != this->getHeight())
    {
        delete [] m_image;
        m_image = 0;
    }
    if(m_image == 0)
    {
        //Allocate memory for array of elements of column
        m_image = new Pixel*[height];
    }
    // Now point the pointers in the right place
    int pixelIndex = 0;
    for( int i = 0; i < height; ++i)
    {
        m_image[i] = &buffer[pixelIndex];
        pixelIndex += width;
    }
    m_imageWidth = width;
    m_imageHeight = height;
}

void NUimage::setImageDimensions(int newWidth, int newHeight)
{
    // If the image size has not changed do nothing.
    if((m_imageWidth == newWidth) && (m_imageHeight == newHeight))
    {
        return;
    }

    if(m_usingInternalBuffer == true)
    {
        removeInternalBuffer();
        addInternalBuffer(newWidth, newHeight);
    }
    m_imageWidth = newWidth;
    m_imageHeight = newHeight;
    return;
}

/*! @brief Put the entire contents of the NUSensorsData class into a stream
 */
std::ostream& operator<< (std::ostream& output, const NUimage& p_image)
{
    int sourceWidth = p_image.getWidth();
    int sourceHeight = p_image.getHeight();
    double timeStamp = p_image.m_timestamp;
    output.write(reinterpret_cast<char*>(&sourceWidth), sizeof(sourceWidth));
    output.write(reinterpret_cast<char*>(&sourceHeight), sizeof(sourceHeight));
    output.write(reinterpret_cast<char*>(&timeStamp), sizeof(timeStamp));
    for(int y = 0; y < sourceHeight; y++)
    {
        output.write(reinterpret_cast<char*>(p_image.m_image[y]), sizeof(p_image.m_image[y][0])*sourceWidth);
    }
    return output;
}

std::istream& operator>> (std::istream& input, NUimage& p_image)
{
    int width, height;
    input.read(reinterpret_cast<char*>(&width), sizeof(width));
    input.read(reinterpret_cast<char*>(&height), sizeof(height));
    input.read(reinterpret_cast<char*>(&p_image.m_timestamp), sizeof(p_image.m_timestamp));

    p_image.setImageDimensions(width, height);
    p_image.useInternalBuffer(true);

    for(int y = 0; y < height; y++)
    {
        input.read((char*) p_image.m_image[y], sizeof(p_image.m_image[y][0])*width);
    }
    return input;
}
