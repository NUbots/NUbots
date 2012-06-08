#include "virtualnubot.h"
#include "Tools/FileFormats/LUTTools.h"
#include <QDebug>
#include <zlib.h>
#include "VisionOld/LineDetection.h"
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
    //! TODO: Load LUT from filename.
    AllObjects = new FieldObjects();
    classificationTable = new unsigned char[LUTTools::LUT_SIZE];
    tempLut = new unsigned char[LUTTools::LUT_SIZE];
    for (int i = 0; i < LUTTools::LUT_SIZE; i++)
    {
        classificationTable[i] = ClassIndex::unclassified;
        tempLut[i] = ClassIndex::unclassified;
    }
    classImage.useInternalBuffer();
    previewClassImage.useInternalBuffer();
    nextUndoIndex = 0;
    rawImage = 0;
    jointSensors = 0;
    balanceSensors = 0;
    touchSensors = 0;

    autoSoftColour = false;

    sensorsData = new NUSensorsData();
    setSensorData(sensorsData);
    
    vision = VisionControlWrapper::getInstance();
    //debug<<"VirtualNUBot started";
    //TEST:

}

virtualNUbot::~virtualNUbot()
{
    delete classificationTable;
}

void virtualNUbot::setRawImage(const NUImage* image)
{
    rawImage = image;
    vision.setImage(image);
    return;
}


void virtualNUbot::setSensorData(const float* joint, const float* balance, const float* touch)
{
    jointSensors = joint;
    balanceSensors = balance;
    touchSensors = touch;
    horizonLine.Calculate(balanceSensors[4],balanceSensors[3],jointSensors[0],jointSensors[1],cameraNumber);
    emit lineDisplayChanged(&horizonLine, GLDisplay::horizonLine);

}
void virtualNUbot::setSensorData(NUSensorsData* newsensorsData)
{

    //std::stringstream data;
    //newsensorsData->summaryTo(data);
    //qDebug() << data.str().c_str() << endl;

    sensorsData = newsensorsData;
    vision->setSensorData(sensorsData);
    vector<float> horizondata;
    bool isOK = sensorsData->getHorizon(horizondata);
    if(isOK)
    {
        horizonLine.setLine((double)horizondata[0],(double)horizondata[1],(double)horizondata[2]);
    }
    else
    {
        horizonLine.setLine((double)horizondata[0],(double)horizondata[1],(double)horizondata[2]);
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
    vision.classifyImage(classImage);
    emit classifiedDisplayChanged(&classImage, GLDisplay::classifiedImage);
    return;
}


void virtualNUbot::processVisionFrame()
{
    processVisionFrame(rawImage);
}

void virtualNUbot::processVisionFrame(const NUImage* image)
{

    vision->setSensorsData(sensorsData);
    vision->setFieldObjects(AllObjects);
    vision->setImage(image);
    
    vision.setLUT(classificationTable);
    
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


    //POST PROCESS:
    vision.AllFieldObjects->postProcess(image->GetTimestamp());
    qDebug() << image->GetTimestamp() ;
    emit candidatesDisplayChanged(candidates, GLDisplay::ObjectCandidates);
    emit fieldObjectsChanged(vision.AllFieldObjects);
    emit fieldObjectsDisplayChanged(vision.AllFieldObjects,GLDisplay::FieldObjects);

    //SUMMARY:
    qDebug() << "Time: " << vision.m_timestamp;
    for(unsigned int i = 0; i < vision.AllFieldObjects->stationaryFieldObjects.size();i++)
    {
        if(vision.AllFieldObjects->stationaryFieldObjects[i].isObjectVisible() == true)
        {
            qDebug() << "Stationary Object: " << i << ":" << QString(vision.AllFieldObjects->stationaryFieldObjects[i].getName().c_str())
                     <<"Seen at "<<  vision.AllFieldObjects->stationaryFieldObjects[i].ScreenX()
                     <<","       <<  vision.AllFieldObjects->stationaryFieldObjects[i].ScreenY()
                    << "\t Distance: " << vision.AllFieldObjects->stationaryFieldObjects[i].measuredDistance();
        }
    }
    for(unsigned  int i = 0; i < vision.AllFieldObjects->mobileFieldObjects.size();i++)
    {
        if(vision.AllFieldObjects->mobileFieldObjects[i].isObjectVisible() == true)
        {
            qDebug() << "Mobile Object: " << i << ":" << QString(vision.AllFieldObjects->mobileFieldObjects[i].getName().c_str())
                     << "Seen at "   <<  vision.AllFieldObjects->mobileFieldObjects[i].ScreenX()
                     <<","           <<  vision.AllFieldObjects->mobileFieldObjects[i].ScreenY()
                    << "\t Distance: " << vision.AllFieldObjects->mobileFieldObjects[i].measuredDistance();
        }
    }

    for(unsigned int i = 0; i < vision.AllFieldObjects->ambiguousFieldObjects.size();i++)
    {
        if(vision.AllFieldObjects->ambiguousFieldObjects[i].isObjectVisible() == true)
        {
            qDebug() << "Ambiguous Object: " << i << ":" << vision.AllFieldObjects->ambiguousFieldObjects[i].getID()
                     <<  QString(vision.AllFieldObjects->ambiguousFieldObjects[i].getName().c_str())
                     << "Seen at "          <<  vision.AllFieldObjects->ambiguousFieldObjects[i].ScreenX()
                     << ","                 <<  vision.AllFieldObjects->ambiguousFieldObjects[i].ScreenY()
                     << "\t Distance: " << vision.AllFieldObjects->ambiguousFieldObjects[i].measuredDistance();

        }
    }

    return;
}

void virtualNUbot::processVisionFrame(ClassifiedImage& image)
{
    return;
}


void virtualNUbot::updateSelection(ClassIndex::Colour colour, std::vector<Pixel> indexs)
{
    if(!imageAvailable()) return;
    Pixel temp;
    float LUTSelectedCounter[ClassIndex::num_colours+1];

    //Set colour counters to 0;
    for (int col = 0; col < ClassIndex::num_colours+1; col++)
    {
        LUTSelectedCounter[col] = 0;
    }

    // Add selected values to temporary lookup table.
    for (unsigned int i = 0; i < indexs.size(); i++)
    {
        temp = indexs[i];
        unsigned int index = LUTTools::getLUTIndex(temp);
        LUTSelectedCounter[ClassIndex::Colour(classificationTable[index])] = LUTSelectedCounter[ClassIndex::Colour(classificationTable[index])] +1;
        tempLut[index] = getUpdateColour(ClassIndex::Colour(classificationTable[index]),colour);
    }

    //Send Stats to Classification widget to display
    emit updateStatistics(LUTSelectedCounter);

    // Create Classifed Image based on lookup table.
    vision.classifyPreviewImage(previewClassImage,tempLut);

    // Remove selection from temporary lookup table.
    for (unsigned int i = 0; i < indexs.size(); i++)
    {
        temp = indexs[i];
        unsigned int index = LUTTools::getLUTIndex(temp);
        tempLut[index] = ClassIndex::unclassified;
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


void virtualNUbot::UpdateLUT(ClassIndex::Colour colour, std::vector<Pixel> indexs)
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
            classificationTable[index] = getUpdateColour(ClassIndex::Colour(classificationTable[index]),colour);
        }
    }
    nextUndoIndex++;
    if(nextUndoIndex >= maxUndoLength)
        nextUndoIndex = 0;
    processVisionFrame(rawImage);
    emit LUTChanged(classificationTable);
    return;
}

ClassIndex::Colour virtualNUbot::getUpdateColour(ClassIndex::Colour currentColour, ClassIndex::Colour requestedColour)
{
    if(autoSoftColour == false) return requestedColour;
    switch(currentColour)
    {
        case ClassIndex::pink:
        {
            switch(requestedColour)
            {
            case ClassIndex::orange:
            case ClassIndex::pink_orange:
                return ClassIndex::pink_orange;
                break;
            default:
                return requestedColour;
                break;
            }
            break;
        }
        case ClassIndex::pink_orange:
        {
            switch(requestedColour)
            {
            case ClassIndex::pink:
            case ClassIndex::orange:
            case ClassIndex::pink_orange:
                return ClassIndex::pink_orange;
                break;
            default:
                return requestedColour;
                break;
            }
            break;
        }
        case ClassIndex::orange:
        {
            switch(requestedColour)
            {
            case ClassIndex::pink:
            case ClassIndex::pink_orange:
                return ClassIndex::pink_orange;
                break;
            case ClassIndex::yellow:
            case ClassIndex::yellow_orange:
                return ClassIndex::yellow_orange;
                break;
            default:
                return requestedColour;
                break;
            }
            break;
        }
        case ClassIndex::yellow_orange:
        {
            switch(requestedColour)
            {
            case ClassIndex::yellow:
            case ClassIndex::orange:
            case ClassIndex::yellow_orange:
                return ClassIndex::yellow_orange;
                break;
            default:
                return requestedColour;
                break;
            }
            break;
        }
        case ClassIndex::yellow:
        {
            switch(requestedColour)
            {
            case ClassIndex::orange:
            case ClassIndex::yellow_orange:
                return ClassIndex::yellow_orange;
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

