#ifndef VIRTUALNUBOT_H
#define VIRTUALNUBOT_H

#include <QObject>
#include <QImage>
#include "Tools/FileFormats/NUbotImage.h"
#include "Infrastructure/NUImage/NUImage.h"
#include "Vision/ClassificationColours.h"
#include "classificationwidget.h"
#include "Kinematics/Horizon.h"
#include "Infrastructure/NUImage/ClassifiedImage.h"
#include "GLDisplay.h"
#include "Vision/Vision.h"
#include "Tools/Math/LSFittedLine.h"
#include "Vision/CornerPoint.h"
#include "Vision/Circle.h"
#include <vector>
#include <fstream>
#include "FileAccess/LogFileReader.h"
#include "debugverbositynetwork.h"

class NUBlackboard;

#define uint8 unsigned char


/**
 *A Structure that defines a Classified Packet.
 */
struct ClassifiedPacketData{
    int frameNumber;            //!< Current Frame Number on the Robot
    int frameWidth;             //!< Image Width
    int frameHeight;            //!< Image Height
    uint8 classImage[76800];    //!< Array of unsigned 8-bit integers containing the classified image
};


class virtualNUbot : public QObject
{
Q_OBJECT
public:
    virtualNUbot(QObject * parent = 0);
    ~virtualNUbot();
    Pixel selectRawPixel(int x, int y);
    bool imageAvailable()
    {
        return (rawImage != 0);
    }

    QString fileType;
public slots:
    /** Processes a Classified Image Packet, to be displayed in program
    *  @param datagram The classified image packet that is recieved, and to be processed by program for visualisation and further vision processing
    */
    void ProcessPacket(QByteArray* packet);
    void updateLookupTable(unsigned char* packetBuffer){return;}
    void updateSelection(ClassIndex::Colour colour, std::vector<Pixel> indexs);
    void UpdateLUT(ClassIndex::Colour colour, std::vector<Pixel> indexs);
    void UndoLUT();
    void saveLookupTableFile(QString fileName);
    void loadLookupTableFile(QString fileName);

    void setRawImage(const NUImage* image);
    void setSensorData(const float* joint, const float* balance, const float* touch);
    void setSensorData(NUSensorsData* NUSensorsData);
    void setCamera(int newCamera){cameraNumber = newCamera;};
    void setAutoSoftColour(bool isEnabled){autoSoftColour = isEnabled;};
    void processVisionFrame();

signals:
    void imageDisplayChanged(const NUImage* updatedImage, GLDisplay::display displayId);
    void classifiedDisplayChanged(ClassifiedImage* updatedImage, GLDisplay::display displayId);
    void lineDisplayChanged(Line* line, GLDisplay::display displayId);
    void cornerPointsDisplayChanged(std::vector< CornerPoint> corners, GLDisplay::display displayId );
    void pointsDisplayChanged(std::vector< Vector2<int> > updatedPoints, GLDisplay::display displayId);
    void transitionSegmentsDisplayChanged(std::vector< TransitionSegment > updatedTransitionSegments, GLDisplay::display displayId);
    void lineDetectionDisplayChanged(std::vector<LSFittedLine> fieldLines, GLDisplay::display displayId);
    void linePointsDisplayChanged(std::vector<LinePoint> linepoints, GLDisplay::display displayId);
    void candidatesDisplayChanged(std::vector< ObjectCandidate > updatedCandidates, GLDisplay::display displayId);
    void fieldObjectsDisplayChanged(FieldObjects* AllFieldObjects, GLDisplay::display displayId);
    void fieldObjectsChanged(const FieldObjects* AllFieldObjects);
    void edgeFilterChanged(QImage image, GLDisplay::display displayId);
    void fftChanged(QImage image, GLDisplay::display displayId);
    void updateStatistics(float* selectedColourCounters);
    void LUTChanged(unsigned char* classificationTable);

private:
    class classEntry
    {
        public:
        classEntry(): index(0), colour(0){}
        classEntry(int newIndex, unsigned char newColour): index(newIndex), colour(newColour){}
        int index;
        unsigned char colour;
    };
/**ADDED BY SHANNON**/
    void printPoints(const vector<LinePoint>& points) const;
    void printCandidates(const vector<ObjectCandidate>& candidates, ofstream& file) const;
    void printOtherPoints(const vector<LinePoint>& points, ofstream& file) const;
/**ADDED BY SHANNON**/

    void processVisionFrame(const NUImage* image);
    void processVisionFrame(ClassifiedImage& image);

    void generateClassifiedImage();
    ClassIndex::Colour getUpdateColour(ClassIndex::Colour currentColour, ClassIndex::Colour requestedColour);

    unsigned char* classificationTable;
    unsigned char* tempLut;
    bool autoSoftColour;
    // Data Storage
    const NUImage* rawImage;

    ClassifiedImage classImage, previewClassImage;
    Vision vision;
    FieldObjects* AllObjects;
    int cameraNumber;
    Horizon horizonLine;
    NUBlackboard* m_blackboard;
    NUSensorsData* sensorsData;
    const float* jointSensors;
    const float* balanceSensors;
    const float* touchSensors;
    static const int maxUndoLength = 10;
    int nextUndoIndex;
    std::vector<classEntry> undoHistory[maxUndoLength];
};

#endif // VIRTUALNUBOT_H
