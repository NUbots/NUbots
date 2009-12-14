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
    std::vector< Vector2<int> > horizontalPoints;
    int spacings = 8;
    switch (image.imageFormat)
    {
        case pixels::YUYV:
            generateClassifiedImage(image);

            // Find the green edges
            points = vision.findGreenBorderPoints(&image,classificationTable,spacings,&horizonLine);
            emit pointsDisplayChanged(points,GLDisplay::greenHorizonScanPoints);

            // Find the Field border
            points = vision.getConvexFieldBorders(points);
            points = vision.interpolateBorders(points,spacings);
            emit pointsDisplayChanged(points,GLDisplay::greenHorizonPoints);

            // Scan Image
            verticalPoints = vision.verticalScan(points,spacings);
            horizontalPoints = vision.horizontalScan(points,spacings);

            qDebug()<< (verticalPoints.size() + horizontalPoints.size()) * 100/(image.height()*image.width()) << " percent of image";
            emit pointsDisplayChanged(horizontalPoints,GLDisplay::horizontalScanPath);
            emit pointsDisplayChanged(verticalPoints,GLDisplay::verticalScanPath);
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
    if(hasImage == false) return;
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
