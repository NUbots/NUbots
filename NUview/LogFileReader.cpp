#include "LogFileReader.h"
#include <QStringList>
#include <QDebug>

LogFileReader::LogFileReader(QObject *parent) :
    QObject(parent)
{
    closeFile();
}

LogFileReader::~LogFileReader()
{
    closeFile();
}

QString LogFileReader::getFileTypeFromName(const QString& fileName)
{
    QStringList list = fileName.split('.'); // Split filename by '.'
    return list.last(); // return the last part of the filename (extension)
}

int LogFileReader::calculateFramesInStream(ifstream& stream)
{
    int x, y, fileLength, numImages, startingPos;

    startingPos = stream.tellg();               // save current position
    stream.seekg(0,ios_base::end);              // move to end of file
    fileLength = stream.tellg();                // see where that is
    stream.seekg(0,ios_base::beg);              // move back to start
    stream >> x >> y;                           // read in data
    stream.seekg(startingPos,ios_base::beg);    // return to original position

    // Calculate the number of frames.
    numImages = fileLength / (2*sizeof(x) + x*y*sizeof(int) + 3*sizeof(' '));
    return numImages;
}

QString LogFileReader::getErrorFlagsFromStream(ifstream& stream)
{
    QString messageLine = "streamFile: %1 bit set.\n";
    QString message;
    if(stream.bad())
        message += message.arg("bad");
    if(stream.eof())
        message += message.arg("eof");
    if(stream.fail())
        message += message.arg("fail");
    return message;
}

void LogFileReader::emitControlAvailability()
{
    bool next = false;
    bool prev = false;
    bool first = false;
    bool last = false;
    bool select = false;

    if(fileOpen)
    {
        if(currentFileExtension == QString("nif"))  // ".nif" file
        {
            next = true;
            prev = true;
            first = true;
            last = true;
            select = true;
        }
        else if(currentFileExtension == QString("nul"))   // Open ".nul" stream file
        {
            next = true;
            prev = false;
            first = true;
            last = false;
            select = false;
        }
    }
    emit nextFrameAvailable(next);
    emit previousFrameAvailable(prev);
    emit firstFrameAvailable(first);
    emit lastFrameAvailable(last);
    emit setFrameAvailable(select);
    return;
}

bool LogFileReader::closeFile()
{
    // close any files open
    currentNifFile.closeFile();
    currentFileStream.close();
    fileOpen = false;

    // Clear all tracking variables.
    totalFrames = 0;
    currentFrameIndex = 0;
    fileLengthInBytes = 0;
    currentFileName.clear();
    currentFileExtension.clear();
    emitControlAvailability();
    emit fileClosed();
    return true;
}

int LogFileReader::openFile(QString fileName)
{    
    // Close any currently open file
    closeFile();

    // Store current file name / type
    currentFileName = fileName;
    currentFileExtension = getFileTypeFromName(fileName);

    if(currentFileExtension == QString("nif"))  // Open ".nif" file
    {
        int numFiles = currentNifFile.openFile(fileName.toStdString().c_str(), false);
        if(numFiles != -1)
        {
            fileOpen = true;
            emitControlAvailability();
            emit fileOpened(fileName);
        }
        return numFiles;
    }
    else if(currentFileExtension == QString("nul"))   // Open ".nul" stream file
    {
        currentFileStream.open(fileName.toStdString().c_str(),ios_base::in|ios_base::binary);
        currentFileStream.seekg(0,ios_base::end);
        fileLengthInBytes = currentFileStream.tellg();
        currentFileStream.seekg(0,ios_base::beg);
        if(currentFileStream.is_open() && currentFileStream.good())
        {
            fileOpen = true;
            emitControlAvailability();
            emit fileOpened(fileName);
            return calculateFramesInStream(currentFileStream);
        }
    }
    return -1;
}


int LogFileReader::nextFrame()
{
    if(!fileOpen) return 0;
    int newFrame = currentFrame()+1;
    return setFrame(newFrame);
}

int LogFileReader::previousFrame()
{
    if(!fileOpen) return 0;
    int newFrame = currentFrame()-1;
    return setFrame(newFrame);
}

int LogFileReader::firstFrame()
{
    if(!fileOpen) return 0;
    return setFrame(1);
}

int LogFileReader::lastFrame()
{
    if(!fileOpen) return 0;
    return setFrame(numFrames());
}

int LogFileReader::setFrame(int frameNumber)
{
    if(!fileOpen) return 0;
    bool validFrame = (frameNumber > 0) && (frameNumber <= numFrames());
    if(validFrame)
    {
        NaoCamera camera;
        if(fileType() == QString("nif"))
        {
            uint8 imgbuffer[320*240*2];
            int robotFrameNumber;

            currentNifFile.getImageFrame(frameNumber, robotFrameNumber, camera, imgbuffer, jointSensorsBuffer, balanceSensorsBuffer, touchSensorsBuffer);
            rawImageBuffer.CopyFromYUV422Buffer(imgbuffer,320,240);
        }
        else if (fileType() == QString("nul"))
        {
            if(frameNumber == 1)
            {
                currentFileStream.seekg(0,ios_base::beg);
                currentFileStream.clear();
            }
            if(currentFileStream.good()){
                unsigned int currPos = currentFileStream.tellg();
                if( (fileLengthInBytes - currPos) > 2*sizeof(int) )
                {
                    currentFileStream >> rawImageBuffer;
                }
            }
            else
            {
                qDebug() << "Error Reading File " << fileName() << ":\n" << getErrorFlagsFromStream(currentFileStream);
                closeFile();
            }
        }
    }
    emit frameLoadingCompleted(currentFrame(),numFrames());
    return currentFrame();
}
