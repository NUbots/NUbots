#include "SplitStreamFileFormatReader.h"
#include <QDebug>

SplitStreamFileFormatReader::SplitStreamFileFormatReader(QObject *parent): LogFileFormatReader(parent)
{
    setKnownDataTypes();
}

SplitStreamFileFormatReader::SplitStreamFileFormatReader(const QString& filename, QObject *parent): LogFileFormatReader(parent)
{
    setKnownDataTypes();
    openFile(filename);
}

SplitStreamFileFormatReader::~SplitStreamFileFormatReader()
{
    closeFile();
}

void SplitStreamFileFormatReader::setKnownDataTypes()
{
    m_dataIsSynced = true;
    m_extension = ".strm";
    m_knownDataTypes << "image" << "sensor" << "locwm" << "object" << "locWmFrame";
}

std::vector<QFileInfo> SplitStreamFileFormatReader::FindValidFiles(const QDir& directory)
{
    std::vector<QFileInfo> fileLocations;

    qDebug() << "Set Directory: " << directory.path();

    QStringList extensionsFilter;
    extensionsFilter << "*"+m_extension;
    QStringList files = directory.entryList(extensionsFilter);
    qDebug() << "Num Files = " << files.size();

    // Display files
    QStringList::const_iterator constIterator;
    for (constIterator = files.constBegin(); constIterator != files.constEnd();
           ++constIterator)
        qDebug() << (*constIterator).toLocal8Bit().constData();

    QRegExp rx;
    int index;
    rx.setCaseSensitivity(Qt::CaseInsensitive);

    QStringList::const_iterator constFormatIterator;
    QString displayName;
    for (constFormatIterator = m_knownDataTypes.constBegin(); constFormatIterator != m_knownDataTypes.constEnd();
           ++constFormatIterator)
    {
        rx.setPattern((*constFormatIterator) + m_extension);
        index = files.indexOf(rx);
        if(index != -1)
        {
            fileLocations.push_back(QFileInfo(directory, files[index]));
            displayName = (*constFormatIterator);
            displayName[0] = displayName[0].toUpper();
            qDebug() << displayName.toLocal8Bit().constData() << "file present: " << fileLocations.back().filePath();
        }
    }
    return fileLocations;
}

int SplitStreamFileFormatReader::openFile(const QString& filename)
{
    QFileInfo fileInfo(filename);
    QDir openDir;
    openDir.setPath(fileInfo.path());
    if(m_directory.exists())
    {
        if(!fileInfo.fileName().isEmpty() && m_knownDataTypes.contains(fileInfo.baseName(),Qt::CaseInsensitive))
        {
            qDebug() << "Primary File selected: " << fileInfo.filePath();
            m_primaryData = fileInfo.baseName().toLower();
            qDebug() << "Primary data type: " << m_primaryData;
            m_fileInformation = fileInfo;
        }

        // close existing files;
        closeFile();

        std::vector<QFileInfo> fileLocations = FindValidFiles(openDir);
        std::vector<QFileInfo>::const_iterator fileIt;
        QString temp;
        bool successIndicator = false;
        for (fileIt = fileLocations.begin(); fileIt != fileLocations.end(); ++fileIt)
        {
            temp = (*fileIt).baseName();
            if(temp.compare(QString("image"),Qt::CaseInsensitive) == 0)
            {
                qDebug() << "Opening " << (*fileIt).filePath();
                successIndicator = imageReader.OpenFile((*fileIt).filePath().toStdString());
                qDebug() << successIndicator;
                if(successIndicator)
                {
                    qDebug() << imageReader.TotalFrames() << "images found.";
                }
            }
            if(temp.compare(QString("sensor"),Qt::CaseInsensitive) == 0)
            {
                qDebug() << "Opening " << (*fileIt).filePath();
                successIndicator = sensorReader.OpenFile((*fileIt).filePath().toStdString());
                qDebug() << successIndicator;
                if(successIndicator)
                {
                    qDebug() << sensorReader.TotalFrames() << "sensor frames found.";
                }
            }
            if(temp.compare(QString("locwm"),Qt::CaseInsensitive) == 0)
            {
                qDebug() << "Opening " << (*fileIt).filePath();
                successIndicator = locwmReader.OpenFile((*fileIt).filePath().toStdString());
                qDebug() << successIndicator;
                if(successIndicator)
                {
                    qDebug() << locwmReader.TotalFrames() << "locwm frames found.";
                }
            }
            if(temp.compare(QString("locwmframe"),Qt::CaseInsensitive) == 0)
            {
                qDebug() << "Opening " << (*fileIt).filePath();
                successIndicator = locmframeReader.OpenFile((*fileIt).filePath().toStdString());
                qDebug() << successIndicator;
                if(successIndicator)
                {
                    qDebug() << locmframeReader.TotalFrames() << "locwm frames found.";
                }
            }
        }

        m_directory = openDir;
    }
    if(imageReader.IsValid())
    {
        m_totalFrames = imageReader.TotalFrames();
    }
    else if(locwmReader.IsValid())
    {
        m_totalFrames = locwmReader.TotalFrames();
    }
    return m_totalFrames;
}

bool SplitStreamFileFormatReader::closeFile()
{
    imageReader.CloseFile();
    m_totalFrames = 0;
    m_currentFrameIndex = 0;
    return true;
}

bool SplitStreamFileFormatReader::isNextFrameAvailable()
{
    return (m_totalFrames > 0) && (m_currentFrameIndex < m_totalFrames);
}

bool SplitStreamFileFormatReader::isPreviousFrameAvailable()
{
    return (m_totalFrames > 0) && (m_currentFrameIndex > 1);
}

bool SplitStreamFileFormatReader::isFirstFrameAvailable()
{
    return (m_totalFrames > 0);
}

bool SplitStreamFileFormatReader::isLastFrameAvailable()
{
    return (m_totalFrames > 0);
}

bool SplitStreamFileFormatReader::isSetFrameAvailable()
{
    return (m_totalFrames > 0);
}

int SplitStreamFileFormatReader::nextFrame()
{
    return setFrame(++m_currentFrameIndex);
}

int SplitStreamFileFormatReader::previousFrame()
{    
    return setFrame(--m_currentFrameIndex);
}

int SplitStreamFileFormatReader::firstFrame()
{
    return setFrame(1);
}

int SplitStreamFileFormatReader::lastFrame()
{
    return setFrame(m_totalFrames);
}

int SplitStreamFileFormatReader::setFrame(int frameNumber)
{
    if(m_dataIsSynced)
    {
        if(imageReader.IsValid())
        {
            emit rawImageChanged(imageReader.ReadFrameNumber(frameNumber));
            m_currentFrameIndex = imageReader.CurrentFrameSequenceNumber();
        }
        if(sensorReader.IsValid())
        {
            emit sensorDataChanged(sensorReader.ReadFrameNumber(frameNumber));
            m_currentFrameIndex = sensorReader.CurrentFrameSequenceNumber();
        }
        if(locwmReader.IsValid())
        {
            emit LocalisationDataChanged(locwmReader.ReadFrameNumber(frameNumber));
            m_currentFrameIndex = locwmReader.CurrentFrameSequenceNumber();
        }
        //qDebug() << "Set Frame " << frameNumber << "at" << m_currentFrameIndex;
        //m_currentFrameIndex = imageReader.CurrentFrameSequenceNumber();
        emit frameChanged(m_currentFrameIndex, m_totalFrames);
    }
    return m_currentFrameIndex;
}
