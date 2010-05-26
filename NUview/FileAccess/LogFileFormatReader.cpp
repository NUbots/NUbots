#include "LogFileFormatReader.h"
#include <QMessageBox>
#include <QStringList>


LogFileFormatReader::LogFileFormatReader(QObject *parent) :
    QObject(parent)
{
}

LogFileFormatReader::~LogFileFormatReader()
{
}

void LogFileFormatReader::displayControlError(const QString& controlName)
{
    QString message = "Control %1 is not available for this file.";
    QMessageBox::warning( 0, "LogFileFormatReader Error", message.arg(controlName));
}

int LogFileFormatReader::nextFrame()
{
    displayControlError("nextFrame");
    return 0;
}

int LogFileFormatReader::previousFrame()
{
    displayControlError("previousFrame");
    return 0;
}

int LogFileFormatReader::firstFrame()
{
    displayControlError("firstFrame");
    return 0;
}

int LogFileFormatReader::lastFrame()
{
    displayControlError("lastFrame");
    return 0;
}

int LogFileFormatReader::setFrame(int frameNumber)
{
    displayControlError("setFrame");
    return 0;
}
