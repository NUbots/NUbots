#include "virtualnubot.h"
#include "Tools/FileFormats/LUTTools.h"
#include <QDebug>
#include <zlib.h>
#include "../Vision/LineDetection.h"
#include <QDebug>
#include <QStringList>
#include <iostream>
#include <fstream>
#include <qmessagebox.h>
#include "../NUPlatform/NUSensors/NUSensorsData.h"

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
    //debug<<"VirtualNUBot started";
    //TEST:

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
void virtualNUbot::setSensorData(NUSensorsData* newensorsData)
{
    sensorsData = newensorsData;
    vector<float> horizondata;
    bool isOK = sensorsData->getHorizon(horizondata);
    if(isOK)
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
}

Pixel virtualNUbot::selectRawPixel(int x, int y)
{
    if(x < rawImage->getWidth() && y < rawImage->getHeight() && imageAvailable())
    {
        return rawImage->m_image[y][x];
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
    qDebug() << "Begin Process Frame";
    std::vector< Vector2<int> > points;
    std::vector< Vector2<int> > verticalPoints;
    std::vector< TransitionSegment > verticalsegments;
    std::vector< TransitionSegment > horizontalsegments;
    std::vector< TransitionSegment > allsegments;
    std::vector< ObjectCandidate > candidates;

    std::vector< Vector2<int> > horizontalPoints;
    //std::vector<LSFittedLine> fieldLines;


    int tempNumScanLines = 0;

    std::vector<unsigned char> validColours;
    Vision::tCLASSIFY_METHOD method;
    const int ROBOTS = 0;
    const int BALL   = 1;
    const int YELLOW_GOALS  = 2;
    const int BLUE_GOALS  = 3;

    int mode  = ROBOTS;
    Circle circ;
    vision.setSensorsData(sensorsData);
    vision.setFieldObjects(AllObjects);
    vision.setImage(image);
    //qDebug() <<  "Image Set" << image->m_timestamp << vision.AllFieldObjects->stationaryFieldObjects.size();
    vision.AllFieldObjects->preProcess(image->m_timestamp);
    //qDebug() << "Image Pre-processed";
    int spacings = (int)image->getWidth()/20;
    //qDebug() << "Scan Spacing calculated";
    vision.setLUT(classificationTable);
    //qDebug() << "LUT set";
    generateClassifiedImage(image);
    //qDebug() << "Generate Classified Image: finnished";

    //! Find the green edges
    std::vector< Vector2<int> > greenPoints = vision.findGreenBorderPoints(spacings,&horizonLine);
    emit pointsDisplayChanged(points,GLDisplay::greenHorizonScanPoints);
    //qDebug() << "Find Edges: finnished";
    //! Find the Field border
    std::vector< Vector2<int> > boarderPoints = vision.getConvexFieldBorders(greenPoints);
    std::vector< Vector2<int> > interpolatedBoarderPoints = vision.interpolateBorders(boarderPoints,spacings);
    emit pointsDisplayChanged(interpolatedBoarderPoints,GLDisplay::greenHorizonPoints);
    //qDebug() << "Find Field border: finnished";
    //! Scan Below Horizon Image
    ClassifiedSection vertScanArea = vision.verticalScan(interpolatedBoarderPoints,spacings);
    //! Scan Above the Horizon
    ClassifiedSection horiScanArea = vision.horizontalScan(interpolatedBoarderPoints,spacings);
    //qDebug() << "Generate Scanlines: finnished";
    //! Classify Line Segments

    vision.ClassifyScanArea(&vertScanArea);
    vision.ClassifyScanArea(&horiScanArea);
    //qDebug() << "Classify Scanlines: finnished";


    std::vector< TransitionSegment > GoalBlueSegments;
    std::vector< TransitionSegment > GoalYellowSegments;
    std::vector< TransitionSegment > BallSegments;

    //! Extract and Display Vertical Scan Points:
    tempNumScanLines = vertScanArea.getNumberOfScanLines();
    for (int i = 0; i < tempNumScanLines; i++)
    {
        ScanLine* tempScanLine = vertScanArea.getScanLine(i);
        int lengthOfLine = tempScanLine->getLength();
        Vector2<int> startPoint = tempScanLine->getStart();
        for(int seg = 0; seg < tempScanLine->getNumberOfSegments(); seg++)
        {
            verticalsegments.push_back((*tempScanLine->getSegment(seg)));
            allsegments.push_back((*tempScanLine->getSegment(seg)));

            if(     tempScanLine->getSegment(seg)->getColour() == ClassIndex::blue || tempScanLine->getSegment(seg)->getColour() == ClassIndex::shadow_blue)
            {
                GoalBlueSegments.push_back((*tempScanLine->getSegment(seg)));
            }
            if(     tempScanLine->getSegment(seg)->getColour() == ClassIndex::yellow || tempScanLine->getSegment(seg)->getColour() == ClassIndex::yellow_orange)
            {
                GoalYellowSegments.push_back((*tempScanLine->getSegment(seg)));
            }
            if(     tempScanLine->getSegment(seg)->getColour() == ClassIndex::orange || tempScanLine->getSegment(seg)->getColour() == ClassIndex::yellow_orange
                ||  tempScanLine->getSegment(seg)->getColour() == ClassIndex::pink_orange)
            {
                BallSegments.push_back((*tempScanLine->getSegment(seg)));
            }
        }
        if(vertScanArea.getDirection() == ScanLine::DOWN)
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
    tempNumScanLines = horiScanArea.getNumberOfScanLines();
    for (int i = 0; i < tempNumScanLines; i++)
    {
        ScanLine* tempScanLine = horiScanArea.getScanLine(i);
        int lengthOfLine = tempScanLine->getLength();
        //qDebug() << "Amount of fill on scanline["<<i<<"] = "<< tempScanLine->getFill();
        Vector2<int> startPoint = tempScanLine->getStart();
        for(int seg = 0; seg < tempScanLine->getNumberOfSegments(); seg++)
        {
            if(tempScanLine->getSegment(seg)->getColour() == ClassIndex::white) continue;
            horizontalsegments.push_back((*tempScanLine->getSegment(seg)));
            allsegments.push_back((*tempScanLine->getSegment(seg)));
        }
        if(horiScanArea.getDirection() == ScanLine::RIGHT)
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


    emit pointsDisplayChanged(horizontalPoints,GLDisplay::horizontalScanPath);
    emit pointsDisplayChanged(verticalPoints,GLDisplay::verticalScanPath);
    //qDebug() << "disaplay scanPaths: finnished";

    emit transitionSegmentsDisplayChanged(allsegments,GLDisplay::TransitionSegments);

    LineDetection LineDetector;
    vision.DetectLineOrRobotPoints(&vertScanArea,&LineDetector);
    qDebug() << "Number Of Robot Segments: "<<  LineDetector.robotSegments.size() << "Number Of LinePoints: " << LineDetector.linePoints.size() ;

    //! Identify Field Objects
    //qDebug() << "PREclassifyCandidates";



    std::vector< ObjectCandidate > RobotCandidates;
    std::vector< ObjectCandidate > BallCandidates;
    std::vector< ObjectCandidate > BlueGoalCandidates;
    std::vector< ObjectCandidate > YellowGoalCandidates;
    std::vector< ObjectCandidate > BlueGoalAboveHorizonCandidates;
    std::vector< ObjectCandidate > YellowGoalAboveHorizonCandidates;
    mode = BALL;
    method = Vision::PRIMS;
   for (int i = 0; i < 4; i++)
    {

        switch (i)
        {
            case ROBOTS:
                validColours.clear();
                validColours.push_back(ClassIndex::white);
                validColours.push_back(ClassIndex::pink);
                validColours.push_back(ClassIndex::pink_orange);
                validColours.push_back(ClassIndex::shadow_blue);
                //validColours.push_back(ClassIndex::blue);

                qDebug() << "PRE-ROBOT";
                RobotCandidates = vision.classifyCandidates(LineDetector.robotSegments, interpolatedBoarderPoints,validColours, spacings, 0.2, 2.0, 12, method);
                qDebug() << "POST-ROBOT";

                break;
        case BALL:
                validColours.clear();
                validColours.push_back(ClassIndex::orange);
                validColours.push_back(ClassIndex::pink_orange);
                validColours.push_back(ClassIndex::yellow_orange);

                //qDebug() << "PRE-BALL";
                BallCandidates = vision.classifyCandidates(BallSegments, interpolatedBoarderPoints, validColours, spacings, 0, 3.0, 1, method);
                //qDebug() << "POST-BALL";

                break;
            case YELLOW_GOALS:
                validColours.clear();
                validColours.push_back(ClassIndex::yellow);
                validColours.push_back(ClassIndex::yellow_orange);

                //qDebug() << "PRE-GOALS";
                YellowGoalCandidates = vision.classifyCandidates(GoalYellowSegments, interpolatedBoarderPoints, validColours, spacings, 0.1, 4.0, 1, method);
                YellowGoalAboveHorizonCandidates = vision.ClassifyCandidatesAboveTheHorizon(horizontalsegments,validColours,spacings*1.5,3);
                //qDebug() << "POST-GOALS" << tempCandidates.size();

                break;
            case BLUE_GOALS:
                validColours.clear();
                validColours.push_back(ClassIndex::blue);
                //validColours.push_back(ClassIndex::shadow_blue);

                //qDebug() << "PRE-GOALS";
                BlueGoalCandidates = vision.classifyCandidates(GoalBlueSegments, interpolatedBoarderPoints, validColours, spacings, 0.1, 4.0, 1, method);
                BlueGoalAboveHorizonCandidates = vision.ClassifyCandidatesAboveTheHorizon(horizontalsegments,validColours,spacings*1.5,3);
                //qDebug() << "POST-GOALS";

                break;
        }

    }
    //emit candidatesDisplayChanged(candidates, GLDisplay::ObjectCandidates);
    qDebug() << "POSTclassifyCandidates";
    //debug << "POSTclassifyCandidates: " << candidates.size() <<endl;

    //! Find Robots:


    vision.DetectRobots(RobotCandidates);
    candidates.insert(candidates.end(),RobotCandidates.begin(),RobotCandidates.end());

    qDebug() << "Finding YELLOW Goals";
    vision.DetectGoals(YellowGoalCandidates, YellowGoalAboveHorizonCandidates, horizontalsegments);
    candidates.insert(candidates.end(),YellowGoalCandidates.begin(),YellowGoalCandidates.end());
    qDebug() << "Finding BLUE Goals";
    vision.DetectGoals(BlueGoalCandidates, BlueGoalAboveHorizonCandidates,horizontalsegments);
    candidates.insert(candidates.end(),BlueGoalCandidates.begin(),BlueGoalCandidates.end());
     qDebug() << "Finding Lines" ;
    vision.DetectLines(&LineDetector);
    //! Extract Detected Line & Corners
    emit lineDetectionDisplayChanged(LineDetector.fieldLines,GLDisplay::FieldLines);
    emit linePointsDisplayChanged(LineDetector.linePoints,GLDisplay::FieldLines);
    qDebug() << "Updating Corners";
    emit cornerPointsDisplayChanged(LineDetector.cornerPoints,GLDisplay::FieldLines);

     qDebug() << "Finding Balls" << BallCandidates.size();
    if(BallCandidates.size() > 0)
    {
        circ = vision.DetectBall(BallCandidates);
        qDebug() << "Circle found " << circ.isDefined<<": (" << circ.centreX << "," << circ.centreY << ") Radius: "<< circ.radius << " Fitting: " << circ.sd<< endl;
        candidates.insert(candidates.end(),BallCandidates.begin(),BallCandidates.end());
    }

    //POST PROCESS:
    vision.AllFieldObjects->postProcess(image->m_timestamp);
    qDebug() << image->m_timestamp ;
    emit candidatesDisplayChanged(candidates, GLDisplay::ObjectCandidates);
    emit fieldObjectsDisplayChanged(vision.AllFieldObjects,GLDisplay::FieldObjects);

    //SUMMARY:
    qDebug() << "Time: " << vision.m_timestamp;
    qDebug() 	<< "Vision::ProcessFrame - Number of Pixels Classified: " << vision.classifiedCounter
                    << "\t Percent of Image: " << vision.classifiedCounter / float(image->getWidth() * image->getHeight()) * 100.00 << "%" << endl;
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
    // Add selected values to temporary lookup table.
    for (unsigned int i = 0; i < indexs.size(); i++)
    {
        temp = indexs[i];
        unsigned int index = LUTTools::getLUTIndex(temp);
        tempLut[index] = getUpdateColour(ClassIndex::Colour(classificationTable[index]),colour);
    }

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

