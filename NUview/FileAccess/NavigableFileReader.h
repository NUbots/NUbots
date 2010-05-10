#ifndef NAVIGABLEFILEREADER_H
#define NAVIGABLEFILEREADER_H

#include <QObject>

class NavigableFileReader: public QObject
{
    Q_OBJECT
public:
    NavigableFileReader(QObject *parent = 0): QObject(parent){};
    virtual unsigned int TotalFrames() = 0;
    virtual unsigned int CurrentFrameSequenceNumber() = 0;
    virtual void EmitControlAvailability() = 0;
signals:
    void nextFrameAvailable(bool);
    void previousFrameAvailable(bool);
    void firstFrameAvailable(bool);
    void lastFrameAvailable(bool);
    void setFrameAvailable(bool);
};

#endif // NAVIGABLEFILEREADER_H
