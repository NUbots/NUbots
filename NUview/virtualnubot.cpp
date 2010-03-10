#include "virtualnubot.h"
#include "Tools/FileFormats/LUTTools.h"
#include <QDebug>
#include <zlib.h>
#include "../Vision/LineDetection.h"

#include <QStringList>
#include <iostream>
#include <fstream>
#include <qmessagebox.h>

virtualNUbot::virtualNUbot(QObject * parent): QObject(parent)
{
    //! TODO: Load LUT from filename.
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
    //debug<<"VirtualNUBot started";
}

virtualNUbot::~virtualNUbot()
{
    delete classificationTable;
}

void virtualNUbot::setRawImage(const NUimage* image)
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

void virtualNUbot::saveLookupTableFile(QString fileName)
{
    LUTTools::SaveLUT(classificationTable,LUTTools::LUT_SIZE,fileName.toAscii());
}

void virtualNUbot::loadLookupTableFile(QString fileName)
{
    LUTTools::LoadLUT(classificationTable,LUTTools::LUT_SIZE,fileName.toAscii());
    processVisionFrame();
}

Pixel virtualNUbot::selectRawPixel(int x, int y)
{
    if(x < rawImage->width() && y < rawImage->height() && imageAvailable())
    {
        return rawImage->image[y][x];
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
/*
    //Update Image:
    classifiedImage.height = currentPacket->frameHeight;
    classifiedImage.width = currentPacket->frameWidth;
    classifiedImage.imageBuffer = currentPacket->classImage;
    classifiedImage.imageFormat = NUimage::CLASSIFIED;
    processVisionFrame(classifiedImage);
    emit classifiedImageChanged(&classifiedImage);
    */
}

void virtualNUbot::generateClassifiedImage(const NUimage* yuvImage)
{
    vision.classifyImage(classImage);
    emit classifiedDisplayChanged(&classImage, GLDisplay::classifiedImage);
    return;
}

void virtualNUbot::processVisionFrame()
{
    processVisionFrame(rawImage);
}

void virtualNUbot::processVisionFrame(const NUimage* image)
{
    if(!imageAvailable()) return;
    //qDebug() << "Begin Process Frame";
    std::vector< Vector2<int> > points;
    std::vector< Vector2<int> > verticalPoints;
    std::vector< TransitionSegment > verticalsegments;
    std::vector< TransitionSegment > horzontalsegments;
    std::vector< TransitionSegment > allsegments;
    std::vector< TransitionSegment > segments;
    std::vector< ObjectCandidate > candidates;
    std::vector< ObjectCandidate > tempCandidates;
    ClassifiedSection* vertScanArea = new ClassifiedSection();
    ClassifiedSection* horiScanArea = new ClassifiedSection();
    std::vector< Vector2<int> > horizontalPoints;
    std::vector<LSFittedLine> fieldLines;
    int spacings = 8;

    int tempNumScanLines = 0;
    int robotClassifiedPoints = 0;

    std::vector<unsigned char> validColours;
    Vision::tCLASSIFY_METHOD method;
    const int ROBOTS = 0;
    const int BALL   = 1;
    const int YELLOW_GOALS  = 2;
    const int BLUE_GOALS  = 3;

    int mode  = ROBOTS;
    Circle circ;
    //qDebug() << "Start switch";


    vision.setImage(image);
    vision.setLUT(classificationTable);
    generateClassifiedImage(image);
    //qDebug() << "Generate Classified Image: finnished";

    //! Find the green edges
    points = vision.findGreenBorderPoints(spacings,&horizonLine);
    emit pointsDisplayChanged(points,GLDisplay::greenHorizonScanPoints);
    //qDebug() << "Find Edges: finnished";
    //! Find the Field border
    points = vision.getConvexFieldBorders(points);
    points = vision.interpolateBorders(points,spacings);
    emit pointsDisplayChanged(points,GLDisplay::greenHorizonPoints);
    //qDebug() << "Find Field border: finnished";
    //! Scan Below Horizon Image
    vertScanArea = vision.verticalScan(points,spacings);
    //! Scan Above the Horizon
    horiScanArea = vision.horizontalScan(points,spacings);
    //qDebug() << "Generate Scanlines: finnished";
    //! Classify Line Segments

    vision.ClassifyScanArea(vertScanArea);
    vision.ClassifyScanArea(horiScanArea);
    //qDebug() << "Classify Scanlines: finnished";


    //! Extract and Display Vertical Scan Points:
    tempNumScanLines = vertScanArea->getNumberOfScanLines();
    for (int i = 0; i < tempNumScanLines; i++)
    {
        ScanLine* tempScanLine = vertScanArea->getScanLine(i);
        int lengthOfLine = tempScanLine->getLength();
        Vector2<int> startPoint = tempScanLine->getStart();
        for(int seg = 0; seg < tempScanLine->getNumberOfSegments(); seg++)
        {
            verticalsegments.push_back((*tempScanLine->getSegment(seg)));
            allsegments.push_back((*tempScanLine->getSegment(seg)));
            segments.push_back((*tempScanLine->getSegment(seg)));
        }
        if(vertScanArea->getDirection() == ClassifiedSection::DOWN)
        {
            for(int j = 0;  j < lengthOfLine; j++)
            {
                Vector2<int> temp;
                temp.x = startPoint.x;
                temp.y = startPoint.y + j;
                verticalPoints.push_back(temp);
            }
        }
    }

    //! Extract and Display Horizontal Scan Points:
    tempNumScanLines = horiScanArea->getNumberOfScanLines();
    for (int i = 0; i < tempNumScanLines; i++)
    {
        ScanLine* tempScanLine = horiScanArea->getScanLine(i);
        int lengthOfLine = tempScanLine->getLength();
        Vector2<int> startPoint = tempScanLine->getStart();
        for(int seg = 0; seg < tempScanLine->getNumberOfSegments(); seg++)
        {
            horzontalsegments.push_back((*tempScanLine->getSegment(seg)));
            allsegments.push_back((*tempScanLine->getSegment(seg)));
        }
        if(horiScanArea->getDirection() == ClassifiedSection::RIGHT)
        {
            for(int j = 0;  j < lengthOfLine; j++)
            {
                Vector2<int> temp;
                temp.x = startPoint.x + j;
                temp.y = startPoint.y;
                horizontalPoints.push_back(temp);
            }
        }
    }
    //! Form Lines
    fieldLines = vision.DetectLines(vertScanArea,spacings);
    //! Extract Detected Line & Corners
    emit lineDetectionDisplayChanged(fieldLines,GLDisplay::FieldLines);

    emit pointsDisplayChanged(horizontalPoints,GLDisplay::horizontalScanPath);
    emit pointsDisplayChanged(verticalPoints,GLDisplay::verticalScanPath);
    //qDebug() << "disaplay scanPaths: finnished";

    emit transitionSegmentsDisplayChanged(allsegments,GLDisplay::TransitionSegments);

    //! Identify Field Objects
    //qDebug() << "PREclassifyCandidates";

    /*
    std::vector<ObjectCandidate> classifyCandidates(std::vector< TransitionSegment > segments,
                                                    std::vector<Vector2<int> >&fieldBorders,
                                                    std::vector<unsigned char> validColours,
                                                    int spacing,
                                                    float min_aspect, float max_aspect, int min_segments,
                                                    tCLASSIFY_METHOD method);
    */

    mode = ROBOTS;
    method = Vision::PRIMS;
    for (int i = 0; i < 4; i++)
    {
        validColours.clear();
        switch (i)
        {
            case ROBOTS:
                validColours.push_back(ClassIndex::white);
                validColours.push_back(ClassIndex::red);
                validColours.push_back(ClassIndex::red_orange);
                validColours.push_back(ClassIndex::shadow_blue);
                //qDebug() << "PRE-ROBOT";
                tempCandidates = vision.classifyCandidates(segments, points, validColours, spacings, 0.2, 2.0, 12, method);
                //qDebug() << "POST-ROBOT";
                robotClassifiedPoints = 0;
                break;
            case BALL:
                validColours.push_back(ClassIndex::orange);
                validColours.push_back(ClassIndex::red_orange);
                validColours.push_back(ClassIndex::yellow_orange);
                //qDebug() << "PRE-BALL";
                tempCandidates = vision.classifyCandidates(segments, points, validColours, spacings, 0, 3.0, 1, method);
                //qDebug() << "POST-BALL";
                break;
            case YELLOW_GOALS:
                validColours.push_back(ClassIndex::yellow);
                validColours.push_back(ClassIndex::yellow_orange);
                //qDebug() << "PRE-GOALS";
                tempCandidates = vision.classifyCandidates(segments, points, validColours, spacings, 0.1, 4.0, 2, method);
                //qDebug() << "POST-GOALS";
            case BLUE_GOALS:
                validColours.push_back(ClassIndex::blue);
                validColours.push_back(ClassIndex::shadow_blue);
                //qDebug() << "PRE-GOALS";
                tempCandidates = vision.classifyCandidates(segments, points, validColours, spacings, 0.1, 4.0, 2, method);
                //qDebug() << "POST-GOALS";
                break;
        }
        while (tempCandidates.size())
        {
            candidates.push_back(tempCandidates.back());
            tempCandidates.pop_back();
        }
    }
    emit candidatesDisplayChanged(candidates, GLDisplay::ObjectCandidates);
    //qDebug() << "POSTclassifyCandidates";

    circ = vision.DetectBall(candidates);
    //qDebug() << "Ball Detected:" << vision.AllFieldObjects->mobileFieldObjects[FieldObjects::FO_BALL].isObjectVisible();
    if(circ.isDefined)
    {
        //! Draw Ball:
        emit drawFO_Ball((float)circ.centreX,(float)circ.centreY,(float)circ.radius,GLDisplay::TransitionSegments);
    }
    else
    {
        emit transitionSegmentsDisplayChanged(allsegments,GLDisplay::TransitionSegments);
    }
    //qDebug()<< (double)((double)vision.classifiedCounter/(double)(image.height()*image.width()))*100 << " percent of image classified";
    //emit transitionSegmentsDisplayChanged(allsegments,GLDisplay::TransitionSegments);
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
    // Add selected values to temporary lookup table.
    for (unsigned int i = 0; i < indexs.size(); i++)
    {
        temp = indexs[i];
        unsigned int index = ((temp.y<<16) + (temp.cb<<8) + temp.cr);
        tempLut[index] = getUpdateColour(ClassIndex::Colour(classificationTable[index]),colour);
    }

    // Create Classifed Image based on lookup table.
    vision.classifyPreviewImage(previewClassImage,tempLut);

    // Remove selection from temporary lookup table.
    for (unsigned int i = 0; i < indexs.size(); i++)
    {
        temp = indexs[i];
        unsigned int index = ((temp.y<<16) + (temp.cb<<8) + temp.cr);
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
        unsigned int index = ((temp.y<<16) + (temp.cb<<8) + temp.cr);
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
    return;
}

ClassIndex::Colour virtualNUbot::getUpdateColour(ClassIndex::Colour currentColour, ClassIndex::Colour requestedColour)
{
    if(autoSoftColour == false) return requestedColour;
    switch(currentColour)
    {
        case ClassIndex::red:
        {
            switch(requestedColour)
            {
            case ClassIndex::orange:
            case ClassIndex::red_orange:
                return ClassIndex::red_orange;
                break;
            default:
                return requestedColour;
                break;
            }
            break;
        }
        case ClassIndex::red_orange:
        {
            switch(requestedColour)
            {
            case ClassIndex::red:
            case ClassIndex::orange:
            case ClassIndex::red_orange:
                return ClassIndex::red_orange;
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
            case ClassIndex::red:
            case ClassIndex::red_orange:
                return ClassIndex::red_orange;
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

