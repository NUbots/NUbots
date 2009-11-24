/*!
  @file NUbotImage.h
  @author Steven Nicklin
  @brief Definition of the NUbotImage class
*/

#ifndef NUBOT_IMAGE_H_DEFINED
#define NUBOT_IMAGE_H_DEFINED

#include <iostream>
#include <fstream>
#include "cameraDefinitions.h"

#ifndef uint8
typedef unsigned char uint8;
#endif
using namespace std;

#define CLASSIFIED_BYTES_PER_PIXEL 1
#define YUV_BYTES_PER_PIXEL 2

enum imageFrameType{
    T_CLASSIFIED,
    T_YUV,
    T_NUM_FRAME_TYPES
};

struct imageFileHeader{
    float version;
    int numberOfFrames;
    imageFrameType imageType;
    int imageWidth;
    int imageHeight;
    int numJointSensors;
    int numBalanceSensors;
    int numTouchSensors;
    int imageFrameLength;
};

struct imageFrameHeader{
    int robotFrameNumber;
    NaoCamera cameraID;
};

/*!
  @brief Class used to read and write image and sensor data to a file.
  @todo New file format required. Needs to be flexible with image size as
  well as the number of sensors etc. It would also be good if the data saved
  is configurable so that we only need to store what we are interested in.
  */
class NUbotImage {
    public:
        //! Standard Constructor.
        NUbotImage();
        //! Destructor.
        ~NUbotImage();
    
        fstream currentFile;    //!< The file used to read / write to.
        int currentFileLengthInBytes; //!< The length of the current file.
        imageFileHeader currentFileHeader; //!< the heder information for the current file.
    
        /*!
          @brief Opens a file for reading or writing.
          @param filename The name of the file to open.
          @param overwrite If true any existing data in the file will be cleared.
          @return Returns the number of frames contained within the opened file.
          */
        int openFile(std::string filename, bool overwrite = true);
        /*!
          @brief Closes any open file.
          @return Returns true if a file was closed. False if there was no file open.
          */
        bool closeFile();

        /*!
          @brief Writes image and sensor data to the end of the current file.
          @param robotFrameNumber The number of the frame to be written.
          @param camera The camera data to be written to file.
          @param image The image to be written to file.
          @param jointSensors The joint sensor data to be written.
          @param balanceSensors The balance sensor data to be written.
          @param touchSensors The touch sensor data to be written.
          @return Returns the new number of frames present in the file.
          */
        int appendImageFrame(int robotFrameNumber, NaoCamera camera, uint8* image, float* jointSensors, float* balanceSensors, float* touchSensors);
        /*!
          @brief Read image and sensor data from the current file.
          @param frameId The number of the frame within the file to be read.
          @param robotFrameNumber variable to write the frame number to.
          @param camera Variable to write the camera data to.
          @param image Buffer to write the image data to.
          @param jointSensors Buffer to write the joint sensor data to.
          @param balanceSensors Buffer to write the balance sensor data to.
          @param touchSensors Buffer to write the touch sensor data to.
          @return Returns True if the file was read succesfully.
          */
        bool getImageFrame(  int frameId, int& robotFrameNumber, NaoCamera& camera, 
                            uint8* image, float* jointSensors, float* balanceSensors, float* touchSensors);

        /*!
          @brief Writes image and sensor data provided to the provided buffer.
          @param buffer Th buffer where the data will be writen.
          @param robotFrameNumber The number of the frame to be written.
          @param camera The camera data to be written to file.
          @param image The image to be written to file.
          @param jointSensors The joint sensor data to be written.
          @param balanceSensors The balance sensor data to be written.
          @param touchSensors The touch sensor data to be written.
          @return Returns the new number of frames present in the file.
          */
        int writeToBuffer(  char* buffer, int robotFrameNumber, NaoCamera camera, 
                            uint8* image, float* jointSensors, float* balanceSensors, float* touchSensors);

        /*!
          @brief Read image and sensor data from the given buffer.
          @param buffer The buffer containing the information.
          @param frameId The number of the frame within the file to be read.
          @param robotFrameNumber variable to write the frame number to.
          @param camera Variable to write the camera data to.
          @param image Buffer to write the image data to.
          @param jointSensors Buffer to write the joint sensor data to.
          @param balanceSensors Buffer to write the balance sensor data to.
          @param touchSensors Buffer to write the touch sensor data to.
          @return Returns True if the file was read succesfully.
          */
        bool readFromBuffer(    char* buffer, int& robotFrameNumber, NaoCamera& camera, 
                                uint8* image, float* jointSensors, float* balanceSensors, float* touchSensors);
        /*!
          @brief Get the header data for a particular frame.
          @param frameID the id of the frame whos header data is required.
          @return Returns the requested frame header.
          */
        imageFrameHeader getImageFrameHeader(int frameID);

        /*!
          @brief Calculate the frame length given the dimensions of the data to be stored.
          @param type The type of image to be stored.
          @param imageWidth The width of the image.
          @param imageHeight The hieght of the image.
          @param numJointSensors the number of joint sensors.
          @param numBalanceSensors The number of balance sensors.
          @param numTouchSensors The number of touch sensors.
          @return Returns the total length of the frame data in bytes.
          */
        static int calculateFrameLength(imageFrameType type, int imageWidth, int imageHeight, 
                                        int numJointSensors, int numBalanceSensors, int numTouchSensors);
        /*!
          @brief Calculate the lenght of an image buffer.
          @param type The type of image to be stored.
          @param imageWidth The width of the image.
          @param imageHeight The hieght of the image.
          @return Returns the length of the image buffer described.
          */
        static int calculateImageLength(imageFrameType type, int imageWidth, int imageHeight);
    private:
        imageFileHeader defaultFileHeader;
        int getFrameStartPosition(int frameNumber);
};
#endif
