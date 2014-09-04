#include "NUbotImage.h"
#include <cstring>

NUbotImage::NUbotImage()
{
    currentFileLengthInBytes = 0;
    // Default settings.
    defaultFileHeader.version = 0.0f;
    defaultFileHeader.numberOfFrames = 0;
    defaultFileHeader.imageType = T_YUV;
    defaultFileHeader.imageWidth = 320;
    defaultFileHeader.imageHeight = 240;
    defaultFileHeader.numJointSensors = 22;
    defaultFileHeader.numBalanceSensors = 7;
    defaultFileHeader.numTouchSensors = 13;
    defaultFileHeader.imageFrameLength = calculateFrameLength(  defaultFileHeader.imageType, 
                                                                defaultFileHeader.imageWidth,
                                                                defaultFileHeader.imageHeight,
                                                                defaultFileHeader.numJointSensors,
                                                                defaultFileHeader.numBalanceSensors,
                                                                defaultFileHeader.numTouchSensors);
}



NUbotImage::~NUbotImage()
{
    closeFile();
}


int NUbotImage::calculateFrameLength(imageFrameType type, int imageWidth, int imageHeight, int numJointSensors, int numBalanceSensors, int numTouchSensors)
{
    int imageLength = calculateImageLength(type, imageWidth, imageHeight);
    int totalNumSensors = numJointSensors + numBalanceSensors + numTouchSensors;
    int imageFrameLength = sizeof(imageFrameHeader) + imageLength + totalNumSensors * sizeof(float);
    return imageFrameLength;
}


int NUbotImage::calculateImageLength(imageFrameType type, int imageWidth, int imageHeight)
{
    // Get the bytes per pixel (bpp)
    int bpp;
    if(type == T_CLASSIFIED) bpp = CLASSIFIED_BYTES_PER_PIXEL;
    else if(type == T_YUV) bpp = YUV_BYTES_PER_PIXEL;
    else bpp = 0;

    int imageLength = imageWidth * imageHeight * bpp;
    return imageLength;
}




int NUbotImage::openFile(std::string filename, bool overwrite)
{
    closeFile();
    std::ios_base::openmode mode = std::ios::in | std::ios::out | std::ios::binary | std::ios::ate;
    if(overwrite){
        mode = mode | std::ios::trunc;
    }
    if(!currentFile.is_open()) {
        currentFile.open(filename.c_str(), mode);
    } else {
        return -1;
    }

    // Get the length of the current file
    currentFileLengthInBytes = currentFile.tellg();

    // Move reading pointer back to the beginning of the file.
    currentFile.seekg (0, std::ios::beg);

    int headerLength = sizeof(imageFileHeader);
    if(currentFileLengthInBytes > headerLength){
        struct imageFileHeader header;
        currentFile.read ((char*)&header, headerLength);
        int predictedLength = headerLength + header.imageFrameLength * header.numberOfFrames;
        if(predictedLength != currentFileLengthInBytes){
            std::cout << "ERROR: Invalid file structure." << std::endl;
	    std::cout << "File Length = " << currentFileLengthInBytes << std::endl;
	    std::cout << "File Should have " << header.numberOfFrames << " frames for a total size of " << predictedLength << " bytes." << std::endl;
	    std::cout << "Image Frame Length = " << header.imageFrameLength << std::endl;
            closeFile();
            return -1;
        }
        currentFileHeader = header;
    } 
    else {
	    currentFile.seekp (0, std::ios::beg);
        currentFileHeader = defaultFileHeader;
        currentFile.write ((char*)&currentFileHeader, headerLength);
        currentFile.flush();
    }
    std::cout << "Image Recording File opened: " << filename << std::endl;
    return currentFileHeader.numberOfFrames;
}



bool NUbotImage::closeFile()
{
    bool fileClosed = false;
    if(currentFile.is_open()){
	currentFile.flush();
        currentFile.close();
        fileClosed = true;
    }
    return fileClosed;
}



int NUbotImage::appendImageFrame(int robotFrameNumber, NaoCamera camera, uint8* image, float* jointSensors, float* balanceSensors, float* touchSensors)
{
    if(currentFile.is_open() == false){
	std::cout << "Error no open file." << std::endl;
        return -1;
    }
    currentFile.seekp(0, std::ios::end); // seek to the end of the file.
    imageFrameHeader header;
    header.robotFrameNumber = robotFrameNumber;
    header.cameraID = camera;

    //std::cout << "Appending Image Frame..." << std::endl;

    //std::cout << "Writing frame header" << std::endl;

    // Write the header
    currentFile.write((char*)&header, sizeof(imageFrameHeader));

    //std::cout << "Writing image" << std::endl;

    // Write the image
    int imageLength = calculateImageLength(currentFileHeader.imageType, currentFileHeader.imageWidth, currentFileHeader.imageHeight);
    currentFile.write((char*)image, imageLength);

    //std::cout << "Writing joint positions" << std::endl;
    
    // Write the joint sensors 
    currentFile.write((char*)jointSensors, currentFileHeader.numJointSensors * sizeof(float));

    //std::cout << "Writing balance values" << std::endl;

    // Write balance sensors
    currentFile.write((char*)balanceSensors, currentFileHeader.numBalanceSensors * sizeof(float));
        
    //std::cout << "Writing touch values" << std::endl;

    // Write touch sensors
    currentFile.write((char*)touchSensors, currentFileHeader.numTouchSensors * sizeof(float));

    //std::cout << "Writing joint positions" << std::endl;

    currentFileHeader.numberOfFrames++;

    currentFile.seekp(0, std::ios::beg); // seek to the start of the file.
    //std::cout << "Updating file header" << std::endl;
    currentFile.write ((char*)&currentFileHeader, sizeof(imageFileHeader));

    currentFile.seekp(0, std::ios::end); // seek to the end of the file.
    currentFileLengthInBytes = currentFile.tellp();
    currentFile.flush();
    //std::cout << "Finished!" << std::endl;
    return currentFileHeader.numberOfFrames;
}



bool NUbotImage::getImageFrame(  int frameId, int& robotFrameNumber, NaoCamera& camera, 
                                uint8* image, float* jointSensors, float* balanceSensors, float* touchSensors)
{

    if(currentFile.is_open() == false){
        return false;
    }
    else if (frameId > currentFileHeader.numberOfFrames){
	return false;
    }

    int fileSeekPosition = getFrameStartPosition(frameId);
   
    currentFile.seekg(fileSeekPosition, std::ios::beg); // seek to the frame we are interested in.

    imageFrameHeader header;

    // Read the header
    currentFile.read((char*)&header, sizeof(imageFrameHeader));
//    fileSeekPosition+=sizeof(imageFrameHeader);
    robotFrameNumber = header.robotFrameNumber;
    camera = header.cameraID;

    // Read the image
    int imageLength = calculateImageLength(currentFileHeader.imageType, currentFileHeader.imageWidth, currentFileHeader.imageHeight);
    
//    currentFile.seekg(fileSeekPosition, std::ios::beg); // seek to the frame we are interested in.
    currentFile.read((char*)image, imageLength);
    
    // Read the joint sensorbs 
    currentFile.read((char*)jointSensors, currentFileHeader.numJointSensors * sizeof(float));

    // Read balance sensors
    currentFile.read((char*)balanceSensors, currentFileHeader.numBalanceSensors * sizeof(float));
        
    // Read touch sensors
    currentFile.read((char*)touchSensors, currentFileHeader.numTouchSensors * sizeof(float));

//    currentFile.seekg(0, std::ios::end); // seek to the end of the file.
    return true;
}



int NUbotImage::getFrameStartPosition(int frameId)
{
    int filePosition = sizeof(imageFileHeader) + currentFileHeader.imageFrameLength * (frameId-1);
    return filePosition;
}



int NUbotImage::writeToBuffer(  char* buffer, int robotFrameNumber, NaoCamera camera, 
                            uint8* image, float* jointSensors, float* balanceSensors, float* touchSensors)
{
    imageFrameHeader header;
    header.robotFrameNumber = robotFrameNumber;
    header.cameraID = camera;
    char* bufferStart = buffer;

    // Write the header
    memcpy(buffer, (char*)&header, sizeof(imageFrameHeader));
    buffer += sizeof(imageFrameHeader);

    // Write the image
    int imageLength = calculateImageLength(defaultFileHeader.imageType, defaultFileHeader.imageWidth, defaultFileHeader.imageHeight);
    memcpy(buffer, (char*)image, imageLength);
    buffer += imageLength;
    
    // Write the joint sensors 
    memcpy(buffer, (char*)jointSensors, defaultFileHeader.numJointSensors * sizeof(float));
    buffer += defaultFileHeader.numJointSensors * sizeof(float);

    // Write balance sensors
    memcpy(buffer, (char*)balanceSensors, defaultFileHeader.numBalanceSensors * sizeof(float));
    buffer += defaultFileHeader.numBalanceSensors * sizeof(float);
        
    // Write touch sensors
    memcpy(buffer, (char*)touchSensors, defaultFileHeader.numTouchSensors * sizeof(float));
    buffer += defaultFileHeader.numTouchSensors * sizeof(float);
    return (int)(buffer - bufferStart);
}

bool NUbotImage::readFromBuffer(    char* buffer, int& robotFrameNumber, NaoCamera& camera, 
                                uint8* image, float* jointSensors, float* balanceSensors, float* touchSensors)
{
    imageFrameHeader header;
    //std::cout << "Reading from Header" << sizeof(imageFrameHeader) << std::endl;
    // Read the header
    memcpy((char*)&header, buffer, sizeof(imageFrameHeader));
    buffer += sizeof(imageFrameHeader);
    robotFrameNumber = header.robotFrameNumber;
    camera = header.cameraID;

    // Read the image
    int imageLength = calculateImageLength(defaultFileHeader.imageType, defaultFileHeader.imageWidth, defaultFileHeader.imageHeight);
	 //std::cout << "Reading from Image" << imageLength <<std::endl;
    memcpy((char*)image, buffer, imageLength);
    buffer += imageLength;

    //std::cout << "Reading from Joints" << std::endl;
    // Read the joint sensorbs 
    memcpy((char*)jointSensors, buffer, defaultFileHeader.numJointSensors * sizeof(float));
    buffer += defaultFileHeader.numJointSensors * sizeof(float);

	//std::cout << "Reading from Balance" << std::endl;
    // Read balance sensors
    memcpy((char*)balanceSensors, buffer, defaultFileHeader.numBalanceSensors * sizeof(float));
    buffer += defaultFileHeader.numBalanceSensors * sizeof(float);
        //std::cout << "Reading from Touch" << std::endl;
    // Read touch sensors
    memcpy((char*)touchSensors, buffer, defaultFileHeader.numTouchSensors * sizeof(float));
    buffer += defaultFileHeader.numTouchSensors * sizeof(float);
    return true;
}
