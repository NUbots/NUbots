#include "NUImage.h"
#include <cstring>
#include <string>
#include "ColorModelConversions.h"
/*!
@file NUImage.h
@brief Declaration of NUbots NUImage class. Storage class for images.
*/

NUImage::NUImage(): m_imageWidth(0), m_imageHeight(0), m_usingInternalBuffer(false)
{
    m_image = 0;
}

NUImage::NUImage(int width, int height, bool useInternalBuffer): m_imageWidth(width), m_imageHeight(height), m_usingInternalBuffer(useInternalBuffer)
{
    m_image = 0;
    if(m_usingInternalBuffer)
    {
        addInternalBuffer(width, height);
    }
}

NUImage::NUImage(const NUImage& source): TimestampedData(), m_imageWidth(0), m_imageHeight(0), m_usingInternalBuffer(false)
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

NUImage::~NUImage()
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

void NUImage::copyFromExisting(const NUImage& source)
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

void NUImage::cloneExisting(const NUImage& source)
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
        MapYUV422BufferToImage((unsigned char*)&source.m_image[0][0], sourceWidth*2, sourceHeight*2);
    }
    m_timestamp = source.m_timestamp;
}

void NUImage::useInternalBuffer(bool newCondition)
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

void NUImage::removeInternalBuffer()
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

void NUImage::addInternalBuffer(int width, int height)
{
    Pixel* buffer = allocateBuffer(width, height);
    MapBufferToImage(buffer, width, height);
    m_usingInternalBuffer = true;
}

Pixel* NUImage::allocateBuffer(int width, int height)
{
    Pixel* buffer = new Pixel[width * height];
    m_localBuffer = buffer;
    return buffer;
}

void NUImage::MapYUV422BufferToImage(const unsigned char* buffer, int width, int height)
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

void NUImage::CopyFromYUV422Buffer(const unsigned char* buffer, int width, int height)
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

void NUImage::MapBufferToImage(Pixel* buffer, int width, int height)
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

void NUImage::setImageDimensions(int newWidth, int newHeight)
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

/*
QImage NUImage::getSubImage(int x, int y, int width, int height) const
{
    return getSubImage(x, y, width, height, 1);
}

QImage NUImage::getSubImage(int x, int y, int width, int height, int decimation_spacing) const
{
    //Sanitise inputs
    if (width <= 0  || width  > m_imageWidth)  width  = m_imageWidth;
    if (height <= 0 || height > m_imageHeight) height = m_imageHeight;
    if (x+width > m_imageWidth) x = m_imageWidth - width;
    if (y+height > m_imageHeight) y = m_imageHeight - height;
    if (decimation_spacing < 1) decimation_spacing = 1;

    //qDebug() << "getSubImage ("<<x<<","<<y<<")::w("<<width<<")::h("<<height<<")::iw("<<m_imageWidth<<")::ih("<<m_imageHeight<<")";

    QImage image(width/decimation_spacing, height/decimation_spacing, QImage::Format_ARGB32);
    unsigned int color = 0;
    unsigned char r = 0, g = 0, b = 0;
    Pixel* p;
    //qDebug() << "getSubImage5: "<<decimation_spacing;
    for (int x_ = x; x_ < x+width; x_ += decimation_spacing)
    {
        for (int y_ = y; y_ < y+height; y_ += decimation_spacing)
        {
            //qDebug() << "1 (" << x_ << "," << y_ <<")";
            p = &m_image[y_][x_];
            //qDebug() << "2: "<< p->y << "," << p->cb << "," << p->cr;
            ColorModelConversions::fromYCbCrToRGB( p->y, p->cb, p->cr, r, g, b);
            //qDebug() << "3: " << r << "," << g << "," << b;
            color = (0xff000000) | (r << 16) | (g << 8) | b;
            //qDebug() << "4: alpha=" << (color/16777216);
            image.setPixel(x_ - x, y_ - y, color );
        }
    }
    //qDebug() << "getSubImage6";
    return image;
}*/

/*! @brief Put the entire contents of the NUSensorsData class into a stream
 */
std::ostream& operator<< (std::ostream& output, const NUImage& p_image)
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

std::istream& operator>> (std::istream& input, NUImage& p_image)
{
    int width, height;
    input.read(reinterpret_cast<char*>(&width), sizeof(width));
    input.read(reinterpret_cast<char*>(&height), sizeof(height));
    input.read(reinterpret_cast<char*>(&p_image.m_timestamp), sizeof(p_image.m_timestamp));

    p_image.setImageDimensions(width, height);
    p_image.useInternalBuffer(true);

    for(int y = 0; y < height; y++)
    {
        if(!input.good()) throw std::exception();
        input.read((char*) p_image.m_image[y], sizeof(p_image.m_image[y][0])*width);
    }
    return input;
}
