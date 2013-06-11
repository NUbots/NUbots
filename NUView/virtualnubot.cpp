#include "virtualnubot.h"
#include "Tools/FileFormats/LUTTools.h"
#include <QDebug>
#include <zlib.h>
//#include "VisionOld/LineDetection.h"
#include <QDebug>
#include <QStringList>
#include <iostream>
#include <fstream>
#include <qmessagebox.h>
#include <sstream>

#include "Tools/Profiling/Profiler.h"

#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"

#include "../Kinematics/Kinematics.h"

virtualNUbot::virtualNUbot(QObject * parent): QObject(parent)
{
    vision = VisionControlWrapper::getInstance();

    //make connections
    QObject::connect(vision->wrapper, SIGNAL(pointsUpdated(std::vector<Point>,GLDisplay::display)), this, SIGNAL(pointsDisplayChanged(std::vector<Point>,GLDisplay::display)));
    QObject::connect(vision->wrapper, SIGNAL(linesUpdated(std::vector<LSFittedLine>,GLDisplay::display)), this, SIGNAL(fittedLineDisplayChanged(std::vector<LSFittedLine>,GLDisplay::display)));
    QObject::connect(vision->wrapper, SIGNAL(segmentsUpdated(std::vector<std::vector<ColourSegment> >,GLDisplay::display)), this, SIGNAL(segmentsDisplayChanged(std::vector<std::vector<ColourSegment> >,GLDisplay::display)));
    QObject::connect(vision->wrapper, SIGNAL(plotUpdated(QString, QVector<QPointF>)), this, SIGNAL(curveChanged(QString, QVector<QPointF>)));

    //! TODO: Load LUT from filename.
    AllObjects = new FieldObjects();
    classificationTable = new unsigned char[LUTTools::LUT_SIZE];
    tempLut = new unsigned char[LUTTools::LUT_SIZE];
    for (int i = 0; i < LUTTools::LUT_SIZE; i++)
    {
        classificationTable[i] = Vision::unclassified;
        tempLut[i] = Vision::unclassified;
    }
    classImage.useInternalBuffer();
    previewClassImage.useInternalBuffer();
    nextUndoIndex = 0;
    rawImage = 0;
//    jointSensors = 0;
//    balanceSensors = 0;
//    touchSensors = 0;

    autoSoftColour = false;

    sensorsData = new NUSensorsData();
    setSensorData(sensorsData);
}

virtualNUbot::~virtualNUbot()
{
    delete classificationTable;
}

void virtualNUbot::setRawImage(const NUImage* image)
{
    rawImage = image;
    vision->setRawImage(image);
    return;
}


//void virtualNUbot::setSensorData(const float* joint, const float* balance, const float* touch)
//{
//    jointSensors = joint;
//    balanceSensors = balance;
//    touchSensors = touch;
//    horizonLine.Calculate(balanceSensors[4],balanceSensors[3],jointSensors[0],jointSensors[1],cameraNumber);
//    emit lineDisplayChanged(&horizonLine, GLDisplay::horizonLine);
//}

void virtualNUbot::setSensorData(NUSensorsData* newsensorsData)
{
    //std::stringstream data;
    //newsensorsData->summaryTo(data);
    //qDebug() << data.str().c_str() << std::endl;

    sensorsData = newsensorsData;
    vision->setSensorData(sensorsData);
    std::vector<float> horizondata;
    bool isOK = sensorsData->getHorizon(horizondata);
    if(isOK)
    {
        horizonLine.setLine((double)horizondata[0],(double)horizondata[1],(double)horizondata[2]);
    }
    else
    {
        horizonLine.setLineFromPoints(Vector2<double>(0, 0), Vector2<double>(320,0));
    }
    emit lineDisplayChanged(&horizonLine, GLDisplay::horizonLine);

}

void virtualNUbot::saveLookupTableFile(QString fileName)
{
    LUTTools::SaveLUT(classificationTable,LUTTools::LUT_SIZE,fileName.toAscii());
}

void virtualNUbot::loadLookupTableFile(QString fileName)
{
    LUTTools::LoadLUT(classificationTable,LUTTools::LUT_SIZE,fileName.toAscii());
    processVisionFrame();
    emit LUTChanged(classificationTable);
}

Pixel virtualNUbot::selectRawPixel(int x, int y)
{
    if(x < rawImage->getWidth() && y < rawImage->getHeight() && imageAvailable())
    {
        return (*rawImage)(x,y);
    }
    else
    {
        return Pixel();
    }
}

void virtualNUbot::ProcessPacket(QByteArray* packet)
{
    //qDebug() << "Process Request Recieved";
    int size = 80000;
    int packetOffset = 80000;
    uint8 uncompressed[packetOffset];

    int err = uncompress((Bytef*)uncompressed, (uLong*) &size, (Bytef*)packet->data(), packet->size());

    if (err != 0)
    {
        QString text = QString("ZLIB Error: ");
        text.append(QString::number(err));
        //qDebug() << "Error occured in Extraction: " << text;
        return;
    }

    ClassifiedPacketData* currentPacket = (ClassifiedPacketData*) uncompressed;
    classImage.useInternalBuffer(false);
    classImage.setImageDimensions(currentPacket->frameWidth, currentPacket->frameHeight);
    classImage.MapBufferToImage(currentPacket->classImage,currentPacket->frameWidth, currentPacket->frameHeight);
    emit classifiedDisplayChanged(&classImage, GLDisplay::classifiedImage);
    processVisionFrame(classImage);
}

void virtualNUbot::generateClassifiedImage()
{
    vision->classifyImage(classImage);
    emit classifiedDisplayChanged(&classImage, GLDisplay::classifiedImage);
    return;
}

void virtualNUbot::processVisionFrame()
{
    processVisionFrame(rawImage);
}

void virtualNUbot::processVisionFrame(const NUImage* image)
{
    if(image != NULL) {
        emit clearPlots();
        vision->setSensorData(sensorsData);
        vision->setFieldObjects(AllObjects);
        vision->setRawImage(image);

        vision->setLUT(classificationTable);

        vision->runFrame();

        generateClassifiedImage();

        QImage* canvas = new QImage(image->getWidth(), image->getHeight(), QImage::Format_ARGB32);

        //Blank canvas - zero alpha (transparent)
        for (int x = 0; x < canvas->width(); x++)
            for(int y = 0; y < canvas->height(); y++)
                canvas->setPixel(x,y,0);
        emit edgeFilterChanged(*canvas, GLDisplay::EdgeFilter);

        float datavalue = 0.0;
        sensorsData->get(NUSensorsData::HeadPitch,datavalue);
        qDebug() << "Sensors Data: Head Elevation: " << datavalue;


        FieldObjects* field_objects = vision->wrapper->field_objects;
        //POST PROCESS:
        qDebug() << image->GetTimestamp() ;
        emit fieldObjectsChanged(field_objects);
        emit fieldObjectsDisplayChanged(field_objects,GLDisplay::FieldObjects);

        //SUMMARY:
        qDebug() << "Time: " << vision->wrapper->m_timestamp;
        for(unsigned int i = 0; i < field_objects->stationaryFieldObjects.size();i++)
        {
            if(field_objects->stationaryFieldObjects[i].isObjectVisible() == true)
            {
                qDebug() << "Stationary Object: " << i << ":" << QString(field_objects->stationaryFieldObjects[i].getName().c_str())
                         <<"Seen at "<<  field_objects->stationaryFieldObjects[i].ScreenX()
                         <<","       <<  field_objects->stationaryFieldObjects[i].ScreenY()
                        << "\t Distance: " << field_objects->stationaryFieldObjects[i].measuredDistance();
            }
        }
        for(unsigned  int i = 0; i < field_objects->mobileFieldObjects.size();i++)
        {
            if(field_objects->mobileFieldObjects[i].isObjectVisible() == true)
            {
                qDebug() << "Mobile Object: " << i << ":" << QString(field_objects->mobileFieldObjects[i].getName().c_str())
                         << "Seen at "   <<  field_objects->mobileFieldObjects[i].ScreenX()
                         <<","           <<  field_objects->mobileFieldObjects[i].ScreenY()
                        << "\t Distance: " << field_objects->mobileFieldObjects[i].measuredDistance();
            }
        }

        for(unsigned int i = 0; i < field_objects->ambiguousFieldObjects.size();i++)
        {
            if(field_objects->ambiguousFieldObjects[i].isObjectVisible() == true)
            {
                qDebug() << "Ambiguous Object: " << i << ":" << field_objects->ambiguousFieldObjects[i].getID()
                         <<  QString(field_objects->ambiguousFieldObjects[i].getName().c_str())
                         << "Seen at "          <<  field_objects->ambiguousFieldObjects[i].ScreenX()
                         << ","                 <<  field_objects->ambiguousFieldObjects[i].ScreenY()
                         << "\t Distance: " << field_objects->ambiguousFieldObjects[i].measuredDistance();

            }
        }
    }
}

void virtualNUbot::processVisionFrame(ClassifiedImage& image)
{
    return;
}

void virtualNUbot::updateSelection(Vision::Colour colour, std::vector<Pixel> indexs)
{
    if(!imageAvailable()) return;
    Pixel temp;
    float LUTSelectedCounter[Vision::num_colours+1];

    //Set colour counters to 0;
    for (int col = 0; col < Vision::num_colours+1; col++)
    {
        LUTSelectedCounter[col] = 0;
    }

    // Add selected values to temporary lookup table.
    for (unsigned int i = 0; i < indexs.size(); i++)
    {
        temp = indexs[i];
        unsigned int index = LUTTools::getLUTIndex(temp);
        LUTSelectedCounter[Vision::Colour(classificationTable[index])] = LUTSelectedCounter[Vision::Colour(classificationTable[index])] +1;
        tempLut[index] = getUpdateColour(Vision::Colour(classificationTable[index]),colour);
    }

    //Send Stats to Classification widget to display
    emit updateStatistics(LUTSelectedCounter);

    // Create Classifed Image based on lookup table.
    vision->classifyPreviewImage(previewClassImage,tempLut);

    // Remove selection from temporary lookup table.
    for (unsigned int i = 0; i < indexs.size(); i++)
    {
        temp = indexs[i];
        unsigned int index = LUTTools::getLUTIndex(temp);
        tempLut[index] = Vision::unclassified;
    }
    emit classifiedDisplayChanged(&previewClassImage, GLDisplay::classificationSelection);
}

void virtualNUbot::UndoLUT()
{
    int currIndex = nextUndoIndex - 1;
    if(currIndex < 0) currIndex = maxUndoLength - 1;
    for(unsigned int i = 0; i < undoHistory[currIndex].size(); i++)
    {
        classificationTable[undoHistory[currIndex][i].index] = undoHistory[currIndex][i].colour;
    }
    undoHistory[currIndex].clear();
    std::vector<classEntry>(undoHistory[currIndex]).swap(undoHistory[currIndex]); // Free up vector memory
    nextUndoIndex = currIndex;
    processVisionFrame(rawImage);
}


void virtualNUbot::UpdateLUT(Vision::Colour colour, std::vector<Pixel> indexs)
{
    Pixel temp;
    undoHistory[nextUndoIndex].clear();
    std::vector<classEntry>(undoHistory[nextUndoIndex]).swap(undoHistory[nextUndoIndex]); // Free up vector memory

    for (unsigned int i = 0; i < indexs.size(); i++)
    {
        temp = indexs[i];
        unsigned int index = LUTTools::getLUTIndex(temp);
        if(classificationTable[index] != colour)
        {
            undoHistory[nextUndoIndex].push_back(classEntry(index,classificationTable[index])); // Save index and colour
            classificationTable[index] = getUpdateColour(Vision::Colour(classificationTable[index]),colour);
        }
    }
    nextUndoIndex++;
    if(nextUndoIndex >= maxUndoLength)
        nextUndoIndex = 0;
    processVisionFrame(rawImage);
    emit LUTChanged(classificationTable);
    return;
}

Vision::Colour virtualNUbot::getUpdateColour(Vision::Colour currentColour, Vision::Colour requestedColour)
{
    if(autoSoftColour == false) return requestedColour;
    switch(currentColour)
    {
        case Vision::pink:
        {
            switch(requestedColour)
            {
            case Vision::orange:
            case Vision::pink_orange:
                return Vision::pink_orange;
                break;
            default:
                return requestedColour;
                break;
            }
            break;
        }
        case Vision::pink_orange:
        {
            switch(requestedColour)
            {
            case Vision::pink:
            case Vision::orange:
            case Vision::pink_orange:
                return Vision::pink_orange;
                break;
            default:
                return requestedColour;
                break;
            }
            break;
        }
        case Vision::orange:
        {
            switch(requestedColour)
            {
            case Vision::pink:
            case Vision::pink_orange:
                return Vision::pink_orange;
                break;
            case Vision::yellow:
            case Vision::yellow_orange:
                return Vision::yellow_orange;
                break;
            default:
                return requestedColour;
                break;
            }
            break;
        }
        case Vision::yellow_orange:
        {
            switch(requestedColour)
            {
            case Vision::yellow:
            case Vision::orange:
            case Vision::yellow_orange:
                return Vision::yellow_orange;
                break;
            default:
                return requestedColour;
                break;
            }
            break;
        }
        case Vision::yellow:
        {
            switch(requestedColour)
            {
            case Vision::orange:
            case Vision::yellow_orange:
                return Vision::yellow_orange;
                break;
            default:
                return requestedColour;
                break;
            }
            break;
        }
        default:
            break;

    }
    return requestedColour;
}

