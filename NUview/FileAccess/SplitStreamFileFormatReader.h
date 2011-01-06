#ifndef SPLITSTREAMFILEFORMATREADER_H
#define SPLITSTREAMFILEFORMATREADER_H
#include "LogFileFormatReader.h"
#include "StreamFileReader.h"
#include "Infrastructure/NUImage/NUImage.h"
#include "Localisation/Localisation.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Localisation/LocWmFrame.h"
#include <QDir>

class SplitStreamFileFormatReader: public LogFileFormatReader
{
    Q_OBJECT
public:
    SplitStreamFileFormatReader();
public:
    explicit SplitStreamFileFormatReader(QObject *parent = 0);
    explicit SplitStreamFileFormatReader(const QString& filename, QObject *parent = 0);
    ~SplitStreamFileFormatReader();

    int openFile(const QString& filename);
    bool closeFile();
    bool fileGood(){return true;};

    const NUImage* getImage();

    bool isNextFrameAvailable();
    bool isPreviousFrameAvailable();
    bool isFirstFrameAvailable();
    bool isLastFrameAvailable();
    bool isSetFrameAvailable();

signals:

public slots:
    int nextFrame();
    int previousFrame();
    int firstFrame();
    int lastFrame();
    int setFrame(int frameNumber);

protected:
    std::vector<QFileInfo> FindValidFiles(const QDir& directory);
    void setKnownDataTypes();
    StreamFileReader<NUImage> imageReader;
    StreamFileReader<NUSensorsData> sensorReader;
    StreamFileReader<Localisation> locwmReader;
    StreamFileReader<LocWmFrame> locmframeReader;
    QDir m_directory;
    QStringList m_knownDataTypes;
    QString m_extension;
    QString m_primaryData;
    bool m_dataIsSynced;
};

#endif // SPLITSTREAMFILEFORMATREADER_H
