#include "SplitStreamFileFormatReader.h"
#include <QDebug>

SplitStreamFileFormatReader::SplitStreamFileFormatReader(QObject *parent): LogFileFormatReader(parent)
{
    setKnownDataTypes();
    m_fileGood = false;
}

SplitStreamFileFormatReader::SplitStreamFileFormatReader(const QString& filename, QObject *parent): LogFileFormatReader(parent)
{
    m_fileGood = false;
    setKnownDataTypes();
    openFile(filename);
}

SplitStreamFileFormatReader::~SplitStreamFileFormatReader()
{
    closeFile();
}
const NUImage* SplitStreamFileFormatReader::GetImageData()
{
    return imageReader.ReadFrameNumber(m_currentFrameIndex);
}

const NUSensorsData* SplitStreamFileFormatReader::GetSensorData()
{
    return sensorReader.ReadFrameNumber(m_currentFrameIndex);
}

const Localisation* SplitStreamFileFormatReader::GetLocalisationData()
{
    return locwmReader.ReadFrameNumber(m_currentFrameIndex);
}

const FieldObjects* SplitStreamFileFormatReader::GetObjectData()
{
    return objectReader.ReadFrameNumber(m_currentFrameIndex);
}

void SplitStreamFileFormatReader::setKnownDataTypes()
{
    m_dataIsSynced = true;
    m_extension = ".strm";
    m_knownDataTypes << "image" << "sensor" << "locwm" << "object" << "locWmFrame";

    // Add the file readers.
    m_fileReaders.push_back(&imageReader);
    m_fileReaders.push_back(&sensorReader);
    m_fileReaders.push_back(&locwmReader);
    m_fileReaders.push_back(&objectReader);
    m_fileReaders.push_back(&locmframeReader);
}

std::vector<QFileInfo> SplitStreamFileFormatReader::FindValidFiles(const QDir& directory)
{
    std::vector<QFileInfo> fileLocations;

    qDebug("Searching Path: %s", qPrintable(directory.path()));

    QStringList extensionsFilter;
    extensionsFilter << "*"+m_extension;
    QStringList files = directory.entryList(extensionsFilter);
    qDebug("%d File(s) Found:", files.size());

    // Display files
    QStringList::const_iterator constIterator;
    for (constIterator = files.constBegin(); constIterator != files.constEnd();
           ++constIterator)
        qDebug("%s", qPrintable(*constIterator));

    QRegExp rx;
    int index;
    rx.setCaseSensitivity(Qt::CaseInsensitive);

    QStringList::const_iterator constFormatIterator;
    QString displayName;
    qDebug("Determining File Types:");
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
            qDebug("%s File: %s", qPrintable(displayName), qPrintable(fileLocations.back().filePath()));
        }
    }
    qDebug("Done");
    return fileLocations;
}

int SplitStreamFileFormatReader::openFile(const QString& filename)
{
    // close existing files;
    closeFile();

    QFileInfo fileInfo(filename);
    QDir openDir;
    openDir.setPath(fileInfo.path());

    if(m_directory.exists())
    {
        if(!fileInfo.fileName().isEmpty() && m_knownDataTypes.contains(fileInfo.baseName(),Qt::CaseInsensitive))
        {
            qDebug("Primary File selected: %s", qPrintable(fileInfo.filePath()));
            m_primaryData = fileInfo.baseName().toLower();
            qDebug("Primary data type: %s", qPrintable(m_primaryData));
            m_fileInformation = fileInfo;
        }

        std::vector<QFileInfo> fileLocations = FindValidFiles(openDir);
        std::vector<QFileInfo>::const_iterator fileIt;
        QString temp;
        bool successIndicator = false;
        int index = 0;
        unsigned int totalFrames = 0;
        QString success_msg;
        for (fileIt = fileLocations.begin(); fileIt != fileLocations.end(); ++fileIt)
        {
            temp = (*fileIt).baseName();
            index = m_knownDataTypes.indexOf(temp);
            successIndicator = m_fileReaders[index]->OpenFile((*fileIt).filePath().toStdString());

            if(successIndicator)
            {
                success_msg = QString("Successful! %1 Frames Found.").arg(m_fileReaders[index]->TotalFrames());

            }
            else success_msg = "Failed!";

            qDebug("Opening %s - %s", qPrintable((*fileIt).filePath()), qPrintable(success_msg));

            if((totalFrames == 0) || (totalFrames > m_fileReaders[index]->TotalFrames()))
            {
                totalFrames = m_fileReaders[index]->TotalFrames();
            }
        }
        m_totalFrames = totalFrames;
        m_directory = openDir;
    }
    if(m_totalFrames > 0)
    {
        m_fileGood = true;
        qDebug("Loading complete %d frames available.", m_totalFrames);
    }
    else
    {
        m_fileGood = false;
        qDebug("Loading Failed.");
        m_totalFrames = 0;
    }
    return m_totalFrames;
}

bool SplitStreamFileFormatReader::closeFile()
{
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
