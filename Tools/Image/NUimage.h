/*!
@file NUimage.h 
@brief Declaration of NUbots NUimage class.
@author Steven Nicklin
*/

#ifndef NUIMAGE_H
#define NUIMAGE_H

#include "Pixel.h"
#include <iostream>
#include "Tools/FileFormats/TimestampedData.h"
#include <QImage>

/*!
@brief Class used to store an image and its relevant information.

This is the standard image format that images from different platforms must be
converted to so that they can be used with the NUbot code base. By converting images 
to this standard image format we are able to have our software support multiple robot 
platforms.
*/

class NUimage: public TimestampedData
{
public:
    /*!
    @brief Default constructor.
    */
    NUimage();

    /*!
    @brief Constructor with initial settings.
    @param width Intial image width.
    @param height Intial image height.
    @param useInternalBuffer Initial buffering setting.
    True and internal buffer is created. False it is not.
    */
    NUimage(int width, int height, bool useInternalBuffer);

    /*!
    @brief Copy constructor. Creates a deep copy of the source image.
    @param source The image from which to base the new class.
    */
    NUimage(const NUimage& source);

    /*!
    @brief Destructor.
    */
    ~NUimage();

    /*!
    @brief Creates a local copy of the source image.
    If the source image references an external buffer, a local copy of this buffer will be made.
    @param source The source image.
    */
    void copyFromExisting(const NUimage& source);

    /*!
    @brief Creates an exact copy of the source image.
    If the source image references an external buffer, the clone will reference this same buffer.
    @param source The source image.
    */
    void cloneExisting(const NUimage& source);

    /*!
    @brief Maps a one-dimensional buffer to the two-dimensional image array.
    @param buffer The one-dimensional buffer.
    @param width The width of the image array.
    @param height The height of the image array.
    */
    void MapBufferToImage(Pixel* buffer, int width, int height);

    /*!
    @brief Maps a YUV422 formatted image buffer to the image. A local copy IS NOT made.
    @param buffer The YUV422 image.
    @param width The width of the image.
    @param height The height of the image.
    */
    void MapYUV422BufferToImage(const unsigned char* buffer, int width, int height);

    /*!
    @brief Copies a YUV422 formatted image buffer to the image. A local copy IS made.
    @param buffer The YUV422 image.
    @param width The width of the image.
    @param height The height of the image.
    */
    void CopyFromYUV422Buffer(const unsigned char* buffer, int width, int height);

    /*!
    @brief Output streaming operation.
    @param output The output stream.
    @param p_nuimage The source image to be streamed.
    */
    friend std::ostream& operator<< (std::ostream& output, const NUimage& p_nuimage);

    /*!
    @brief Input streaming operation.
    @param input The input stream.
    @param p_nuimage The destination image to be streamed to.
    */
    friend std::istream& operator>> (std::istream& input, NUimage& p_nuimage);

    /*!
    @brief Get the width of the current image.
    @return The image width.
    */
    int getWidth() const
    {
        return m_imageWidth;
    }
    
    /*!
    @brief Get the height of the current image.
    @return The image height.
    */
    int getHeight() const
    {
        return m_imageHeight;
    }

    /*!
    @brief Get the total number of pixels within the current image.
    @return The number of pixels in the image.
    */
    int getTotalPixels() const
    {
        return getWidth()*getHeight();
    }

    /*!
    @brief Get the current buffering state of the image.
    @return The current buffering state. True when buffered internally. False when buffered externally.
    */
    bool getLocallyBuffered() const
    {
        return m_usingInternalBuffer;
    }

    double GetTimestamp() const
    {
        return m_timestamp;
    }

    /*!
      @brief gets the sub image as defined by the (x,y) of the top left corner and the
      height and width of the rectangle the extends from this origin.
      @param x                  distance from left edge of super-image
      @param y                  distance from top  edge of super-image
      @param width              the width  of the sub image to extract
      @param height             the height of the sub image to extract
      @return returns the subimage from the super image
      */
    QImage getSubImage(int x, int y, int width, int height) const;
    /*!
      @brief gets the sub image as defined by the (x,y) of the top left corner and the
      height and width of the rectangle the extends from this origin.
      @param x                  distance from left edge of super-image
      @param y                  distance from top  edge of super-image
      @param width              the width  of the sub image to extract
      @param height             the height of the sub image to extract
      @param decimation_spacing this is an integer to select every nth pixel in both (x,y)
      @return returns the subimage from the super image
      */
    QImage getSubImage(int x, int y, int width, int height, int decimation_spacing) const;

    Pixel **m_image;                    //!< Pointer to the image array.
    double m_timestamp;			//!< Time point at which the image was captured. (Unix Time)
private:
    int m_imageWidth;                   //!< The current image width.
    int m_imageHeight;                  //!< The current image height.
    bool m_usingInternalBuffer;         //!< The current image buffering state. True when buffered internally. false when buffered externally.
    Pixel *m_localBuffer;               //!< Pointer to the local storage buffer.

    /*!
    @brief Selects the buffering mode for the image.
    @param newCondition Select the new buffering mode. True the image is buffered internally.
    False it is not.
    */
    void useInternalBuffer(bool newCondition = true);

    /*!
    @brief Change the image dimensions.
    @param newWidth The new width of the image.
    @param newHeight The new height of the image.
    */
    void setImageDimensions(int newWidth, int newHeight);

    /*!
    @brief Removes the current internal buffer.
    */
    void removeInternalBuffer();

    /*!
    @brief Adds a new internal buffer of the specified dimensions.
    @param width The width of the new internal buffer.
    @param height The height of the new internal buffer.
    */
    void addInternalBuffer(int width, int height);

    /*!
    @brief Create a buffer of the given dimensions.
    @param width The width of the new buffer.
    @param height The height of the new buffer.
    @return The new buffer.
    */
    Pixel* allocateBuffer(int width, int height);
};
#endif
