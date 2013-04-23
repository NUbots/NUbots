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
    int availableFrames = 0;
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
            //connect(currentFileReader,SIGNAL(sensorDataChanged(const float*, const float*, const float*)),
            //        this, SIGNAL(sensorDataChanged(const float*, const float*, const float*)));
            connect(currentFileReader,SIGNAL(sensorDataChanged( NUSensorsData*)), this, SIGNAL(sensorDataChanged( NUSensorsData*)));
            connect(currentFileReader,SIGNAL(SelfLocalisationDataChanged(const SelfLocalisation*)), this, SIGNAL(SelfLocalisationDataChanged(const SelfLocalisation*)));
            connect(currentFileReader,SIGNAL(ObjectDataChanged(const FieldObjects*)), this, SIGNAL(ObjectDataChanged(const FieldObjects*)));
            connect(currentFileReader,SIGNAL(GameInfoChanged(const GameInformation*)), this, SIGNAL(GameInfoChanged(const GameInformation*)));
            connect(currentFileReader,SIGNAL(TeamInfoChanged(const TeamInformation*)), this, SIGNAL(TeamInfoChanged(const TeamInformation*)));
            connect(currentFileReader,SIGNAL(frameChanged(int,int)), this, SIGNAL(frameChanged(int,int)));
            emit fileOpened(fileName);
            emit AvailableDataChanged(AvailableData());
            emit OpenLogFilesChanged(AvailableLogFiles());
            availableFrames = currentFileReader->numFrames();
        }
        else
        {
            closeFile();
        }
    }
    return availableFrames;
}

bool LogFileReader::closeFile()
{
    if(currentFileReader)
    {
        disconnect(currentFileReader,SIGNAL(rawImageChanged(const NUImage*)), this, SIGNAL(rawImageChanged(const NUImage*)));
        disconnect(currentFileReader,SIGNAL(cameraChanged(int)), this, SIGNAL(cameraChanged(int)));
        //disconnect(currentFileReader,SIGNAL(sensorDataChanged(const float*, const float*, const float*)),
        //        this, SIGNAL(sensorDataChanged(const float*, const float*, const float*)));
        disconnect(currentFileReader,SIGNAL(sensorDataChanged( NUSensorsData*)), this, SIGNAL(sensorDataChanged( NUSensorsData*)));
        disconnect(currentFileReader,SIGNAL(SelfLocalisationDataChanged(const SelfLocalisation*)), this, SIGNAL(SelfLocalisationDataChanged(const SelfLocalisation*)));
        disconnect(currentFileReader,SIGNAL(ObjectDataChanged(const FieldObjects*)), this, SIGNAL(ObjectDataChanged(const FieldObjects*)));
        disconnect(currentFileReader,SIGNAL(frameChanged(int,int)), this, SIGNAL(frameChanged(int,int)));
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
    emit AvailableDataChanged(AvailableData());
    emit OpenLogFilesChanged(AvailableLogFiles());
    return true;
}

int LogFileReader::nextFrame()
{
    if(currentFileReader && currentFileReader->fileGood())
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
    if(currentFileReader && currentFileReader->fileGood())
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
    if(currentFileReader && currentFileReader->fileGood())
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
    if(currentFileReader && currentFileReader->fileGood())
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
    if(currentFileReader && currentFileReader->fileGood())
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
    if(currentFileReader && currentFileReader->fileGood())
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
