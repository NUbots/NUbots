#include "virtualnubot.h"
#include "Tools/FileFormats/LUTTools.h"
#include <QDebug>
#include <zlib.h>

virtualNUbot::virtualNUbot(QObject * parent): QObject(parent)
{
    file = new NUbotImage();
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
    hasImage = false;
}

virtualNUbot::~virtualNUbot()
{
    delete file;
    delete classificationTable;
}

int virtualNUbot::loadFile(const char* fileName)
{
    return file->openFile(fileName, false);
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

pixels::Pixel virtualNUbot::selectRawPixel(int x, int y)
{
    if(x < rawImage.width() && y < rawImage.height() && hasImage)
    {
        return rawImage.image[y][x];
    }
    else
    {
        return pixels::Pixel();
    }
}

void virtualNUbot::loadFrame(int frameNumber)
{
    rawImage.setImageDimensions(160,120);
    rawImage.useInternalBuffer();
    rawImage.imageFormat = pixels::YUYV;

    uint8* rawBuffer = (uint8*)&rawImage.image[0][0];
    NaoCamera camera;
    int robotFrameNumber;

    file->getImageFrame(frameNumber, robotFrameNumber, camera, rawBuffer, jointSensors, balanceSensors, touchSensors);
    hasImage = true;
    horizonLine.Calculate(balanceSensors[4],balanceSensors[3],jointSensors[0],jointSensors[1],camera);
    emit imageDisplayChanged(&rawImage, GLDisplay::rawImage);
    emit lineDisplayChanged(&horizonLine, GLDisplay::horizonLine);
    processVisionFrame(rawImage);
    return;
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
        qDebug() << "Error occured in Extraction: " << text;
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

void virtualNUbot::generateClassifiedImage(const NUimage& yuvImage)
{
    vision.classifyImage(classImage,&rawImage,classificationTable);
    emit classifiedDisplayChanged(&classImage, GLDisplay::classifiedImage);
    return;
}

void virtualNUbot::processVisionFrame()
{
    processVisionFrame(rawImage);
}

void virtualNUbot::processVisionFrame(NUimage& image)
{
    std::vector< Vector2<int> > points;
    std::vector< Vector2<int> > verticalPoints;
    std::vector< TransitionSegment > segments;
    std::vector< RobotCandidate > robotCandidates;
    std::vector< ObjectCandidate > candidates;
    ClassifiedSection* vertScanArea = new ClassifiedSection();
    ClassifiedSection* horiScanArea = new ClassifiedSection();

    std::vector< Vector2<int> > horizontalPoints;

    const int spacings = 8;
    int tempNumScanLines = 0;
    int robotClassifiedPoints = 0;

    std::vector<unsigned char> validColours;
    Vision::tCLASSIFY_METHOD method;
    const int ROBOTS = 0;
    const int BALL   = 1;
    const int GOALS  = 2;
    int mode  = ROBOTS;

    switch (image.imageFormat)
    {
        case pixels::YUYV:
            generateClassifiedImage(image);

            //! Find the green edges
            points = vision.findGreenBorderPoints(&image,classificationTable,spacings,&horizonLine);
            emit pointsDisplayChanged(points,GLDisplay::greenHorizonScanPoints);

            //! Find the Field border
            points = vision.getConvexFieldBorders(points);
            points = vision.interpolateBorders(points,spacings);
            emit pointsDisplayChanged(points,GLDisplay::greenHorizonPoints);

            //! Scan Below Horizon Image
            vertScanArea = vision.verticalScan(points,spacings);
            //! Scan Above the Horizon
            horiScanArea = vision.horizontalScan(points,spacings);

            //! Classify Line Segments

            vision.ClassifyScanArea(vertScanArea);
            vision.ClassifyScanArea(horiScanArea);

            //! Extract and Display Vertical Scan Points:
            tempNumScanLines = vertScanArea->getNumberOfScanLines();
            for (int i = 0; i < tempNumScanLines; i++)
            {
                ScanLine* tempScanLine = vertScanArea->getScanLine(i);
                int lengthOfLine = tempScanLine->getLength();
                Vector2<int> startPoint = tempScanLine->getStart();
                for(int seg = 0; seg < tempScanLine->getNumberOfSegments(); seg++)
                {
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

            emit pointsDisplayChanged(horizontalPoints,GLDisplay::horizontalScanPath);
            emit pointsDisplayChanged(verticalPoints,GLDisplay::verticalScanPath);


            //! Identify Field Objects
            qDebug() << "PREclassifyCandidates";

            mode = ROBOTS;
            method = Vision::PRIMS;
            switch (mode)
            {
                case ROBOTS:
                    validColours.push_back(ClassIndex::white);
                    validColours.push_back(ClassIndex::red);
                    validColours.push_back(ClassIndex::shadow_blue);

                    candidates = vision.classifyCandidates(segments, points, validColours, spacings, 0.1, 2.0, 12, method);

                    robotClassifiedPoints = 0;
/*                    //! Only if there are any robot candidates should further processing continue
                    if (robotCandidates.size() && false)
                    {
                        //segments.clear();
                        std::vector< ClassifiedSection > robotScanAreas = vision.robotScanAreas(robotCandidates, points, horizonLine);
                        std::vector< ClassifiedSection >::iterator nextScanArea = robotScanAreas.begin();
                        for (; nextScanArea != robotScanAreas.end(); nextScanArea++)
                        {
                            tempNumScanLines = nextScanArea->getNumberOfScanLines();
                            for (int i = 0; i < tempNumScanLines; i++)
                            {
                                ScanLine* tempScanLine = nextScanArea->getScanLine(i);
                                robotClassifiedPoints += tempScanLine->getLength();
                                for(int seg = 0; seg < tempScanLine->getNumberOfSegments(); seg++)
                                {
                                    segments.push_back((*tempScanLine->getSegment(seg)));
                                }

                            }
                        }
                    }
//*/
                break;
                case BALL:
                    validColours.push_back(ClassIndex::orange);
                    validColours.push_back(ClassIndex::red_orange);
                    validColours.push_back(ClassIndex::yellow_orange);
                    //validColours.push_back(ClassIndex::red);
                    candidates = vision.classifyCandidates(segments, points, validColours, spacings, 0.3, 3.0, 2, method);
                break;
                case GOALS:
                    validColours.push_back(ClassIndex::yellow);
                    validColours.push_back(ClassIndex::blue);

                    candidates = vision.classifyCandidates(segments, points, validColours, spacings, 0.1, 4.0, 2, method);
                break;
            }


            emit candidatesDisplayChanged(candidates, GLDisplay::ObjectCandidates);
            qDebug() << "POSTclassifyCandidates";


            qDebug()<< (verticalPoints.size() + horizontalPoints.size() + robotClassifiedPoints) * 100/(image.height()*image.width()) << " percent of image classified";
            emit transitionSegmentsDisplayChanged(segments,GLDisplay::TransitionSegments);

            break;
        default:
            break;

    }
    return;
}

void virtualNUbot::processVisionFrame(ClassifiedImage& image)
{
    return;
}


void virtualNUbot::updateSelection(ClassIndex::Colour colour, std::vector<pixels::Pixel> indexs)
{
    pixels::Pixel temp;
    // Add selected values to temporary lookup table.
    for (unsigned int i = 0; i < indexs.size(); i++)
    {
        temp = indexs[i];
        unsigned int index = ((temp.y<<16) + (temp.cb<<8) + temp.cr);
        tempLut[index] = colour;
    }

    // Create Classifed Image based on lookup table.
    vision.classifyImage(previewClassImage,&rawImage,tempLut);

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


void virtualNUbot::UpdateLUT(ClassIndex::Colour colour, std::vector<pixels::Pixel> indexs)
{
    pixels::Pixel temp;
    undoHistory[nextUndoIndex].clear();
    std::vector<classEntry>(undoHistory[nextUndoIndex]).swap(undoHistory[nextUndoIndex]); // Free up vector memory

    for (unsigned int i = 0; i < indexs.size(); i++)
    {
        temp = indexs[i];
        unsigned int index = ((temp.y<<16) + (temp.cb<<8) + temp.cr);
        if(classificationTable[index] != colour)
        {
            undoHistory[nextUndoIndex].push_back(classEntry(index,classificationTable[index])); // Save index and colour
            classificationTable[index] = colour;
        }
    }
    nextUndoIndex++;
    if(nextUndoIndex >= maxUndoLength)
        nextUndoIndex = 0;
    processVisionFrame(rawImage);
    return;
}
