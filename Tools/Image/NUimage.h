/*!
@file NUimage.h 
@brief Declaration of NUbots NUimage class.
@author Steven Nicklin
*/

#ifndef NUIMAGE_H
#define NUIMAGE_H

#include "NUPlatform\NUCamera\NUCameraData.h"
#include "pixels.h"

/*!
@brief Class used to store an image and its relevant information.

This is the standard image format that images from different platforms must be
converted to so that they can be used with the NUbot code base. By converting images 
to this standard image format we are able to have our software support multiple robot 
platforms.
*/

class NUimage
{
public:
    NUimage();
    NUimage(int width, int height, bool useInternalBuffer);
    ~NUimage();

    void useInternalBuffer(bool newCondition = true);
    void removeInternalBuffer();
    void addInternalBuffer(int width, int height);
    pixels::Pixel* allocateBuffer(int width, int height);
    void MapBufferToImage(pixels::Pixel* buffer, int width, int height);
    void setImageDimensions(int newWidth, int newHeight);

    int width() const
    {
        return imageWidth;
    }
    
    int height() const
    {
        return imageHeight;
    }

    int totalPixels() const
    {
        return width()*height();
    }

    pixels::Pixel **image;
    pixels::Pixel *localBuffer;
    pixels::ColourSpace imageFormat;	//!< The colour format of the image
    CameraInfo cameraInfo;		//!< Properties and settings of the camera used to capture the image.
    long int timestamp;			//!< Time point at which the image was captured. (Unix Time)
private:
    int imageWidth;
    int imageHeight;
    bool internalBuffer;
};
#endif
