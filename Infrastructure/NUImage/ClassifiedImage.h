/*!
    @file ClassifiedImage.h
    @author Steven Nicklin
    @brief Declaration of the ClssifiedImage class.
  */

#ifndef CLASSIFIED_IMAGE_H
#define CLASSIFIED_IMAGE_H

/*!
  @brief Class used to store a classified image and its dimensions.
  */
class ClassifiedImage
{
public:
    ClassifiedImage();
    ClassifiedImage(int width, int height, bool useInternalBuffer);
    ~ClassifiedImage();
    void useInternalBuffer(bool newCondition = true);
    void removeInternalBuffer();
    void addInternalBuffer(int width, int height);
    unsigned char* allocateBuffer(int width, int height);
    void MapBufferToImage(unsigned char* buffer, int width, int height);
    void setImageDimensions(int newWidth, int newHeight);

    int width() const
    {
        return imageWidth;
    }

    int height() const
    {
        return imageHeight;
    }

    unsigned char** image;
    unsigned char* localBuffer;
private:
    int imageWidth;
    int imageHeight;
    bool internalBuffer;
};

#endif
