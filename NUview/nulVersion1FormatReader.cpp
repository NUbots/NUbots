#include "nulVersion1FormatReader.h"
#include <QDebug>

nulVersion1FormatReader::nulVersion1FormatReader(QObject *parent) :
    LogFileFormatReader(parent)
{
}

nulVersion1FormatReader::nulVersion1FormatReader(const QString& filename, QObject *parent):
    LogFileFormatReader(parent)
{
    openFile(filename);
    return;
}

nulVersion1FormatReader::~nulVersion1FormatReader()
{
    closeFile();
}

int nulVersion1FormatReader::openFile(const QString& filename)
{
    fileInformation.setFile(filename);
    closeFile();
    fileStream.open(filename.toStdString().c_str(),std::ios_base::in|std::ios_base::binary);
    if(fileGood())
    {
        fileStream.seekg(0,std::ios_base::beg);
        int x,y;
        fileStream >> x >> y;
        fileStream.seekg(0,std::ios_base::beg);
        totalFrames = fileInformation.size() / (2*sizeof(x) + x*y*sizeof(int) + 3*sizeof(' '));
    }
    return numFrames();
}

bool nulVersion1FormatReader::closeFile()
{
    fileStream.close();
    return true;
}

bool nulVersion1FormatReader::isNextFrameAvailable()
{
    return ( fileGood() && (numFrames() > 0) && (currentFrame() < numFrames()) );
}

bool nulVersion1FormatReader::isFirstFrameAvailable()
{
    return (fileGood() && (numFrames() > 0) );
}

int nulVersion1FormatReader::nextFrame()
{
    if(isNextFrameAvailable())
    {

        currentFrameIndex++;
        fileStream >> rawImageBuffer;
        emit rawImageChanged(&rawImageBuffer);
        emit frameChanged(currentFrame(),numFrames());
    }
    return currentFrame();

}

int nulVersion1FormatReader::firstFrame()
{
    currentFrameIndex = 0;
    nextFrame();
    return currentFrame();
}
