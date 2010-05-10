#ifndef IMAGESTREAMFILEREADER_H
#define IMAGESTREAMFILEREADER_H

#include <QObject>
#include "Tools/Image/NUimage.h"

#include "StreamFileReader.h"

class ImageStreamFileReader: public QObject
{
    Q_OBJECT
public:
    ImageStreamFileReader(QObject* parent = 0): QObject(parent){};
    ImageStreamFileReader(const QString& filename, QObject* parent = 0): QObject(parent)
    {
        imageReader.OpenFile(filename.toStdString());
    };
    void OpenFile(const QString& filename)
    {
        imageReader.OpenFile(filename.toStdString());
    };
private:
    StreamFileReader<NUimage> imageReader;
signals:
    void NewDataAvailable(NUimage* newData);

public slots:
    void nextFrame()
    {
        if(imageReader.IsValid())
        {
            NUimage* result = imageReader.ReadNextFrame();
            if(result)
            {
                emit NewDataAvailable(result);
            }
        }
    };
    void previousFrame()
    {
        if(imageReader.IsValid())
        {
            NUimage* result = imageReader.ReadPrevFrame();
            if(result)
            {
                emit NewDataAvailable(result);
            }
        }
    };
    void firstFrame()
    {
        if(imageReader.IsValid())
        {
            NUimage* result = imageReader.ReadFirstFrame();
            if(result)
            {
                emit NewDataAvailable(result);
            }
        }
    };
    void lastFrame()
    {
        if(imageReader.IsValid())
        {
            NUimage* result = imageReader.ReadLastFrame();
            if(result)
            {
                emit NewDataAvailable(result);
            }
        }
    };
    void setFrame(int frameNumber)
    {
        if(imageReader.IsValid())
        {
            NUimage* result = imageReader.ReadFrameNumber(frameNumber);
            if(result)
            {
                emit NewDataAvailable(result);
            }
        }
    };
};

#endif // IMAGESTREAMFILEREADER_H
