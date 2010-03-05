#ifndef NIFVERSION1FORMATREADER_H
#define NIFVERSION1FORMATREADER_H

#include "LogFileFormatReader.h"
#include "Tools/FileFormats/NUbotImage.h"



class nifVersion1FormatReader : public LogFileFormatReader
{
Q_OBJECT
public:
    explicit nifVersion1FormatReader(QObject *parent = 0);
    explicit nifVersion1FormatReader(const QString& filename, QObject *parent = 0);
    int openFile(const QString& filename);
    bool closeFile();
    bool fileGood(){return nifFileReader.currentFile.good();};

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
    NUbotImage nifFileReader;
};

#endif // NIFVERSION1FORMATREADER_H
