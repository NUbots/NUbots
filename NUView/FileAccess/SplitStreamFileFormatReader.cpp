#include "SplitStreamFileFormatReader.h"
#include <QDebug>

SplitStreamFileFormatReader::SplitStreamFileFormatReader(QObject *parent): LogFileFormatReader(parent)
{

    string temp_accel_names[] = {"Gps", "Compass", "Odometry", "Falling", "Fallen", "MotionGetupActive", "LLegEndEffector", "RLegEndEffector"};
    vector<string> sensor_names(temp_accel_names, temp_accel_names + sizeof(temp_accel_names)/sizeof(*temp_accel_names));
    m_tempSensors.addSensors(sensor_names);
    setKnownDataTypes();
    m_fileGood = false;
}

SplitStreamFileFormatReader::SplitStreamFileFormatReader(const QString& filename, QObject *parent): LogFileFormatReader(parent)
{
    string temp_accel_names[] = {"Gps", "Compass", "Odometry", "Falling", "Fallen", "MotionGetupActive", "LLegEndEffector", "RLegEndEffector"};
    vector<string> sensor_names(temp_accel_names, temp_accel_names + sizeof(temp_accel_names)/sizeof(*temp_accel_names));
    m_tempSensors.addSensors(sensor_names);
    m_fileGood = false;
    setKnownDataTypes();
    openFile(filename);
}

SplitStreamFileFormatReader::~SplitStreamFileFormatReader()
{
    closeFile();
}

std::vector<QFileInfo> SplitStreamFileFormatReader::AvailableLogFiles()const
{
    return m_open_files;
}

QStringList SplitStreamFileFormatReader::AvailableData() const
{
    return m_available_data;
}

const NUImage* SplitStreamFileFormatReader::GetImageData()
{
    return imageReader.ReadFrameNumber(m_currentFrameIndex);
}

const NUSensorsData* SplitStreamFileFormatReader::GetSensorData()
{
    if(sensorReader.IsValid())
    {
        return sensorReader.ReadFrameNumber(m_currentFrameIndex);
    }
    else if (locsensorReader.IsValid())
    {
        NULocalisationSensors* temp = locsensorReader.ReadFrameNumber(m_currentFrameIndex);
        if(temp)
        {
            m_tempSensors.setLocSensors(*temp);
            return &m_tempSensors;
        }
    }
    return NULL;
}

const Localisation* SplitStreamFileFormatReader::GetLocalisationData()
{
    return locwmReader.ReadFrameNumber(m_currentFrameIndex);
}

FieldObjects* SplitStreamFileFormatReader::GetObjectData()
{
    return objectReader.ReadFrameNumber(m_currentFrameIndex);
}

const GameInformation* SplitStreamFileFormatReader::GetGameInfo()
{
    return gameinfoReader.ReadFrameNumber(m_currentFrameIndex);
}

const TeamInformation* SplitStreamFileFormatReader::GetTeamInfo()
{
    return teaminfoReader.ReadFrameNumber(m_currentFrameIndex);
}

void SplitStreamFileFormatReader::setKnownDataTypes()
{
    m_dataIsSynced = true;
    m_extension = ".strm";
    m_knownDataTypes.clear();
    m_knownDataTypes << "image" << "sensor" << "locsensor" << "locwm" << "object" << "locWmFrame" << "teaminfo" << "gameinfo";

    // Add the file readers.
    m_fileReaders.push_back(&imageReader);
    m_fileReaders.push_back(&sensorReader);
    m_fileReaders.push_back(&locsensorReader);
    m_fileReaders.push_back(&locwmReader);
    m_fileReaders.push_back(&objectReader);
    m_fileReaders.push_back(&locmframeReader);
    m_fileReaders.push_back(&teaminfoReader);
    m_fileReaders.push_back(&gameinfoReader);
}

std::vector<QFileInfo> SplitStreamFileFormatReader::FindValidFiles(const QDir& directory)
{
    const QString extension = ".strm";
    QStringList knownDataTypes;
    knownDataTypes << "image" << "sensor" << "locsensor" << "locwm" << "object" << "locWmFrame" << "teaminfo" << "gameinfo";
    std::vector<QFileInfo> fileLocations;

    qDebug("Searching Path: %s", qPrintable(directory.path()));

    QStringList extensionsFilter;
    extensionsFilter << "*" + extension;
    QStringList files = directory.entryList(extensionsFilter);
    qDebug("%d File(s) Found:", files.size());

    QRegExp rx;
    int index;
    rx.setCaseSensitivity(Qt::CaseInsensitive);

    QStringList::const_iterator constFormatIterator;
    QString displayName;
    for (constFormatIterator = knownDataTypes.constBegin(); constFormatIterator != knownDataTypes.constEnd();
           ++constFormatIterator)
    {
        rx.setPattern((*constFormatIterator) + extension);
        index = files.indexOf(rx);
        if(index != -1)
        {
            fileLocations.push_back(QFileInfo(directory, files[index]));
            displayName = (*constFormatIterator);
            displayName[0] = displayName[0].toUpper();
//            qDebug("%s File: %s", qPrintable(displayName), qPrintable(fileLocations.back().filePath()));
        }
    }
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
            m_primaryData = fileInfo.baseName().toLower();
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
                m_available_data << m_knownDataTypes[index];
                m_open_files.push_back(*fileIt);
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
        m_available_data.clear();
        m_open_files.clear();
    }
    return m_totalFrames;
}

bool SplitStreamFileFormatReader::closeFile()
{
    m_totalFrames = 0;
    m_currentFrameIndex = 0;
    m_available_data.clear();
    for(int i=0; i < m_fileReaders.size(); i++)
    {
        m_fileReaders[i]->CloseFile();
    }
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
        else if(locsensorReader.IsValid())
        {
            NULocalisationSensors* temp = locsensorReader.ReadFrameNumber(frameNumber);
            if(temp)
            {
                m_tempSensors.setLocSensors(*temp);
                emit sensorDataChanged(&m_tempSensors);
                m_currentFrameIndex = locsensorReader.CurrentFrameSequenceNumber();
            }
        }
        if(locwmReader.IsValid())
        {
            emit LocalisationDataChanged(locwmReader.ReadFrameNumber(frameNumber));
            m_currentFrameIndex = locwmReader.CurrentFrameSequenceNumber();
        }
        if(objectReader.IsValid())
        {
            emit ObjectDataChanged(objectReader.ReadFrameNumber(frameNumber));
            m_currentFrameIndex = objectReader.CurrentFrameSequenceNumber();
        }
        if(teaminfoReader.IsValid())
        {
            emit TeamInfoChanged(teaminfoReader.ReadFrameNumber(frameNumber));
            m_currentFrameIndex = teaminfoReader.CurrentFrameSequenceNumber();
        }
        if(gameinfoReader.IsValid())
        {
            emit GameInfoChanged(gameinfoReader.ReadFrameNumber(frameNumber));
            m_currentFrameIndex = gameinfoReader.CurrentFrameSequenceNumber();
        }
        //qDebug() << "Set Frame " << frameNumber << "at" << m_currentFrameIndex;
        //m_currentFrameIndex = imageReader.CurrentFrameSequenceNumber();
        emit frameChanged(m_currentFrameIndex, m_totalFrames);
    }
    return m_currentFrameIndex;
}
