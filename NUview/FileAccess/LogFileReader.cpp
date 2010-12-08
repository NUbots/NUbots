#include "LogFileReader.h"
#include "nifVersion1FormatReader.h"
#include "nulVersion1FormatReader.h"
#include "SplitStreamFileFormatReader.h"
#include <QFileInfo>
#include <QDebug>
#include "debug.h"

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
    if(fileName.isEmpty()) return 0;
    QString ext = fileInfo.suffix().toLower();
    try{
        if(ext == "nif")
        {
            currentFileReader = new nifVersion1FormatReader(fileName);
        }
        else if(ext == "nul")
        {
           currentFileReader = new nulVersion1FormatReader(fileName);
        }
        else
        {
            currentFileReader = new SplitStreamFileFormatReader(fileName);
        }
    }
    catch(exception &e)
    {
        qDebug() << "I/O Error:" << e.what();
    }

    if(currentFileReader)
    {
        if(currentFileReader->fileGood())
        {
            connect(currentFileReader,SIGNAL(rawImageChanged(const NUImage*)), this, SIGNAL(rawImageChanged(const NUImage*)));
            connect(currentFileReader,SIGNAL(cameraChanged(int)), this, SIGNAL(cameraChanged(int)));
            connect(currentFileReader,SIGNAL(sensorDataChanged(const float*, const float*, const float*)),
                    this, SIGNAL(sensorDataChanged(const float*, const float*, const float*)));
            connect(currentFileReader,SIGNAL(sensorDataChanged( NUSensorsData*)), this, SIGNAL(sensorDataChanged( NUSensorsData*)));
            connect(currentFileReader,SIGNAL(LocalisationDataChanged(const Localisation*)), this, SIGNAL(LocalisationDataChanged(const Localisation*)));
            connect(currentFileReader,SIGNAL(frameChanged(int,int)), this, SIGNAL(frameChanged(int,int)));
            emit fileOpened(fileName);
        }
        else
        {
            closeFile();
        }
    }
    return currentFileReader->numFrames();
}

bool LogFileReader::closeFile()
{
    if(currentFileReader)
    {
        try{
        currentFileReader->closeFile();
        delete currentFileReader;
        currentFileReader = 0;
        }
        catch(exception e)
        {
            qDebug() << "I/O Error: " << e.what();
            closeFile();
        }
    }
    emit fileClosed();
    return true;
}

int LogFileReader::nextFrame()
{
    if(currentFileReader)
    {
        int curr = 0;
        try{
            curr = currentFileReader->nextFrame();
            debug << "Processing Frame: " << curr << endl;
        }
        catch(exception e)
        {
            qDebug() << "I/O Error: " << e.what();
            closeFile();
        }
        emitControlAvailability();

        return curr;

    }
    return 0;
}

int LogFileReader::previousFrame()
{
    if(currentFileReader)
    {
        int curr = 0;
        try{
            curr = currentFileReader->previousFrame();
        }
        catch(exception e)
        {
            qDebug() << "I/O Error: " << e.what();
            closeFile();
        }
        emitControlAvailability();
        return curr;
    }
    return 0;
}

int LogFileReader::firstFrame()
{
    if(currentFileReader)
    {
        int curr = 0;
        try{
            curr = currentFileReader->firstFrame();
        }
        catch(exception e)
        {
            qDebug() << "I/O Error: " << e.what();
            closeFile();
        }

        emitControlAvailability();
        return curr;
    }
    return 0;
}

int LogFileReader::lastFrame()
{
    if(currentFileReader)
    {
        int curr = 0;
        try{
            curr = currentFileReader->lastFrame();
        }
        catch(exception e)
        {
            qDebug() << "I/O Error: " << e.what();
            closeFile();
        }
        emitControlAvailability();
        return curr;
    }
    return 0;
}

int LogFileReader::setFrame(int frameNumber)
{
    if(currentFileReader)
    {
        int curr = 0;
        try{
            curr = currentFileReader->setFrame(frameNumber);
        }
        catch(exception e)
        {
            qDebug() << "I/O Error: " << e.what();
            closeFile();
        }
        emitControlAvailability();
        return curr;
    }
    return 0;
}

void LogFileReader::emitControlAvailability()
{
    if(currentFileReader)
    {
        emit nextFrameAvailable(currentFileReader->isNextFrameAvailable());
        emit previousFrameAvailable(currentFileReader->isPreviousFrameAvailable());
        emit firstFrameAvailable(currentFileReader->isFirstFrameAvailable());
        emit lastFrameAvailable(currentFileReader->isLastFrameAvailable());
        emit setFrameAvailable(currentFileReader->isSetFrameAvailable());
    }
    else
    {
        emit nextFrameAvailable(false);
        emit previousFrameAvailable(false);
        emit firstFrameAvailable(false);
        emit lastFrameAvailable(false);
        emit setFrameAvailable(false);
    }
}
