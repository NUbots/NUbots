#include "LogFileReader.h"
#include "nifVersion1FormatReader.h"
#include <QFileInfo>

LogFileReader::LogFileReader(QObject *parent) :
    QObject(parent)
{
    currentFileReader = 0;
}

LogFileReader::~LogFileReader()
{
    closeFile();
    delete currentFileReader;
    currentFileReader = 0;
}

int LogFileReader::openFile(QString fileName)
{
    closeFile();
    QFileInfo fileInfo(fileName);
    if(fileInfo.suffix().toLower() == QString("nif"))
    {
        currentFileReader = new nifVersion1FormatReader(fileName);
    }

    if(currentFileReader)
    {
        if(currentFileReader->fileGood())
        {
            connect(currentFileReader,SIGNAL(newRawImageAvailable(const NUimage*)), this, SIGNAL(newRawImageAvailable(const NUimage*)));
            connect(currentFileReader,SIGNAL(newSensorDataAvailable(const float*, const float*, const float*)),
                    this, SIGNAL(newSensorDataAvailable(const float*, const float*, const float*)));
            connect(currentFileReader,SIGNAL(frameLoadingCompleted(int,int)),
                    this, SIGNAL(frameLoadingCompleted(int,int)));
            emit fileOpened(fileName);
        }
        else
        {
            delete currentFileReader;
            currentFileReader = 0;
        }
    }
    return currentFileReader->numFrames();
}

bool LogFileReader::closeFile()
{
    if(currentFileReader)
    {
        currentFileReader->closeFile();
        delete currentFileReader;
        currentFileReader = 0;
    }
    emit fileClosed();
    return true;
}

int LogFileReader::nextFrame()
{
    if(currentFileReader)
    {
        return currentFileReader->nextFrame();
    }
    return 0;
}

int LogFileReader::previousFrame()
{
    if(currentFileReader)
    {
        return currentFileReader->previousFrame();
    }
    return 0;
}

int LogFileReader::firstFrame()
{
    if(currentFileReader)
    {
        return currentFileReader->firstFrame();
    }
    return 0;
}

int LogFileReader::lastFrame()
{
    if(currentFileReader)
    {
        return currentFileReader->lastFrame();
    }
    return 0;
}

int LogFileReader::setFrame(int frameNumber)
{
    if(currentFileReader)
    {
        return currentFileReader->setFrame(frameNumber);
    }
    return 0;
}
