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
    ios_base::openmode mode = ios::in | ios::out | ios::binary | ios::ate;
    if(overwrite){
        mode = mode | ios::trunc;
    }
    if(!currentFile.is_open()) {
        currentFile.open(filename.c_str(), mode);
    } else {
        return -1;
    }

    // Get the length of the current file
    currentFileLengthInBytes = currentFile.tellg();

    // Move reading pointer back to the beginning of the file.
    currentFile.seekg (0, ios::beg);

    int headerLength = sizeof(imageFileHeader);
    if(currentFileLengthInBytes > headerLength){
        struct imageFileHeader header;
        currentFile.read ((char*)&header, headerLength);
        int predictedLength = headerLength + header.imageFrameLength * header.numberOfFrames;
        if(predictedLength != currentFileLengthInBytes){
            cout << "ERROR: Invalid file structure." << endl;
	    cout << "File Length = " << currentFileLengthInBytes << endl;
	    cout << "File Should have " << header.numberOfFrames << " frames for a total size of " << predictedLength << " bytes." << endl;
	    cout << "Image Frame Length = " << header.imageFrameLength << endl;
            closeFile();
            return -1;
        }
        currentFileHeader = header;
    } 
    else {
	    currentFile.seekp (0, ios::beg);
        currentFileHeader = defaultFileHeader;
        currentFile.write ((char*)&currentFileHeader, headerLength);
        currentFile.flush();
    }
    cout << "Image Recording File opened: " << filename << endl;
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
	cout << "Error no open file." << endl;
        return -1;
    }
    currentFile.seekp(0, ios::end); // seek to the end of the file.
    imageFrameHeader header;
    header.robotFrameNumber = robotFrameNumber;
    header.cameraID = camera;

    //cout << "Appending Image Frame..." << endl;

    //cout << "Writing frame header" << endl;

    // Write the header
    currentFile.write((char*)&header, sizeof(imageFrameHeader));

    //cout << "Writing image" << endl;

    // Write the image
    int imageLength = calculateImageLength(currentFileHeader.imageType, currentFileHeader.imageWidth, currentFileHeader.imageHeight);
    currentFile.write((char*)image, imageLength);

    //cout << "Writing joint positions" << endl;
    
    // Write the joint sensors 
    currentFile.write((char*)jointSensors, currentFileHeader.numJointSensors * sizeof(float));

    //cout << "Writing balance values" << endl;

    // Write balance sensors
    currentFile.write((char*)balanceSensors, currentFileHeader.numBalanceSensors * sizeof(float));
        
    //cout << "Writing touch values" << endl;

    // Write touch sensors
    currentFile.write((char*)touchSensors, currentFileHeader.numTouchSensors * sizeof(float));

    //cout << "Writing joint positions" << endl;

    currentFileHeader.numberOfFrames++;

    currentFile.seekp(0, ios::beg); // seek to the start of the file.
    //cout << "Updating file header" << endl;
    currentFile.write ((char*)&currentFileHeader, sizeof(imageFileHeader));

    currentFile.seekp(0, ios::end); // seek to the end of the file.
    currentFileLengthInBytes = currentFile.tellp();
    currentFile.flush();
    //cout << "Finished!" << endl;
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
   
    currentFile.seekg(fileSeekPosition, ios::beg); // seek to the frame we are interested in.

    imageFrameHeader header;

    // Read the header
    currentFile.read((char*)&header, sizeof(imageFrameHeader));
//    fileSeekPosition+=sizeof(imageFrameHeader);
    robotFrameNumber = header.robotFrameNumber;
    camera = header.cameraID;

    // Read the image
    int imageLength = calculateImageLength(currentFileHeader.imageType, currentFileHeader.imageWidth, currentFileHeader.imageHeight);
    
//    currentFile.seekg(fileSeekPosition, ios::beg); // seek to the frame we are interested in.
    currentFile.read((char*)image, imageLength);
    
    // Read the joint sensorbs 
    currentFile.read((char*)jointSensors, currentFileHeader.numJointSensors * sizeof(float));

    // Read balance sensors
    currentFile.read((char*)balanceSensors, currentFileHeader.numBalanceSensors * sizeof(float));
        
    // Read touch sensors
    currentFile.read((char*)touchSensors, currentFileHeader.numTouchSensors * sizeof(float));

//    currentFile.seekg(0,ios::end); // seek to the end of the file.
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
    //cout << "Reading from Header" << sizeof(imageFrameHeader) << endl;
    // Read the header
    memcpy((char*)&header, buffer, sizeof(imageFrameHeader));
    buffer += sizeof(imageFrameHeader);
    robotFrameNumber = header.robotFrameNumber;
    camera = header.cameraID;

    // Read the image
    int imageLength = calculateImageLength(defaultFileHeader.imageType, defaultFileHeader.imageWidth, defaultFileHeader.imageHeight);
	 //cout << "Reading from Image" << imageLength <<endl;
    memcpy((char*)image, buffer, imageLength);
    buffer += imageLength;

    //cout << "Reading from Joints" << endl;
    // Read the joint sensorbs 
    memcpy((char*)jointSensors, buffer, defaultFileHeader.numJointSensors * sizeof(float));
    buffer += defaultFileHeader.numJointSensors * sizeof(float);

	//cout << "Reading from Balance" << endl;
    // Read balance sensors
    memcpy((char*)balanceSensors, buffer, defaultFileHeader.numBalanceSensors * sizeof(float));
    buffer += defaultFileHeader.numBalanceSensors * sizeof(float);
        //cout << "Reading from Touch" << endl;
    // Read touch sensors
    memcpy((char*)touchSensors, buffer, defaultFileHeader.numTouchSensors * sizeof(float));
    buffer += defaultFileHeader.numTouchSensors * sizeof(float);
    return true;
}
