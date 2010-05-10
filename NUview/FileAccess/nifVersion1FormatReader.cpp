#include "nifVersion1FormatReader.h"

nifVersion1FormatReader::nifVersion1FormatReader(QObject *parent) :
    LogFileFormatReader(parent)
{
}

nifVersion1FormatReader::nifVersion1FormatReader(const QString& filename, QObject *parent):
    LogFileFormatReader(parent)
{
    openFile(filename);
    return;
}

nifVersion1FormatReader::~nifVersion1FormatReader()
{
    closeFile();
}

int nifVersion1FormatReader::openFile(const QString& filename)
{
    fileInformation.setFile(filename);
    closeFile();
    totalFrames = nifFileReader.openFile(filename.toAscii().data(), false);
    return totalFrames;
}

bool nifVersion1FormatReader::closeFile()
{
    return nifFileReader.closeFile();
}

bool nifVersion1FormatReader::isNextFrameAvailable()
{
    return ( fileGood() && (numFrames() > 0) && (currentFrame() < numFrames()) );
}

bool nifVersion1FormatReader::isPreviousFrameAvailable()
{
    return ( fileGood() && (numFrames() > 0) && (currentFrame() > 1) );
}

bool nifVersion1FormatReader::isFirstFrameAvailable()
{
    return (fileGood() && (numFrames() > 0) );
}

bool nifVersion1FormatReader::isLastFrameAvailable()
{
    return (fileGood() && (numFrames() > 0) );
}

bool nifVersion1FormatReader::isSetFrameAvailable()
{
    return (fileGood() && (numFrames() > 0) );
}

int nifVersion1FormatReader::nextFrame()
{
    int nextFrame = currentFrameIndex + 1;
    return setFrame(nextFrame);
}

int nifVersion1FormatReader::previousFrame()
{
    int prevFrame = currentFrameIndex - 1;
    return setFrame(prevFrame);
}

int nifVersion1FormatReader::firstFrame()
{
    return setFrame(1);
}

int nifVersion1FormatReader::lastFrame()
{
    return setFrame(numFrames());
}

int nifVersion1FormatReader::setFrame(int frameNumber)
{
    if( (frameNumber > 0) && (frameNumber <= numFrames()) && fileGood())
    {
        currentFrameIndex = frameNumber;
        uint8 imgbuffer[320*240*2];
        int robotFrameNumber;
        NaoCamera camera;
        nifFileReader.getImageFrame(frameNumber, robotFrameNumber, camera, imgbuffer, jointSensorsBuffer, balanceSensorsBuffer, touchSensorsBuffer);
        rawImageBuffer.CopyFromYUV422Buffer(imgbuffer,320,240);
        emit cameraChanged(camera);
        emit rawImageChanged(&rawImageBuffer);
        emit sensorDataChanged(jointSensorsBuffer, balanceSensorsBuffer, touchSensorsBuffer);
        emit frameChanged(currentFrame(),numFrames());
    }
    return currentFrame();
}
