#ifndef VIRTUALNUBOT_H
#define VIRTUALNUBOT_H

#include <QObject>
#include "Tools/FileFormats/NUbotImage.h"
#include "Tools/Image/NUimage.h"
#include "Vision/ClassificationColours.h"
#include "classificationwidget.h"
#include "Kinematics/Horizon.h"
#include "Tools/Image/ClassifiedImage.h"
#include "GLDisplay.h"
#include "Vision/Vision.h"
#include "Tools/Math/LSFittedLine.h"
#include <vector>


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
    void loadFrame(int FrameNumber);
    int loadFile(const char* fileName);
    pixels::Pixel selectRawPixel(int x, int y);
    bool imageAvailable()
    {
        return hasImage;
    }

public slots:
    /** Processes a Classified Image Packet, to be displayed in program
    *  @param datagram The classified image packet that is recieved, and to be processed by program for visualisation and further vision processing
    */
    void ProcessPacket(QByteArray* packet);
    void updateLookupTable(unsigned char* packetBuffer){return;}
    void updateSelection(ClassIndex::Colour colour, std::vector<pixels::Pixel> indexs);
    void UpdateLUT(ClassIndex::Colour colour, std::vector<pixels::Pixel> indexs);
    void UndoLUT();
    void saveLookupTableFile(QString fileName);
    void loadLookupTableFile(QString fileName);    

signals:
    void imageDisplayChanged(NUimage* updatedImage, GLDisplay::display displayId);

    /*!
      @brief Sends the robot data to the localisation widget
      @param joints The angles of the robot's joints.
      @param bottomCamera The camera being used.
      @param touch The values of the robot's touch sensors.
      */
    void imageDisplayChanged(double* joints,bool bottomCamera,double * touch);

    void classifiedDisplayChanged(ClassifiedImage* updatedImage, GLDisplay::display displayId);
    void lineDisplayChanged(Line* line, GLDisplay::display displayId);
    void pointsDisplayChanged(std::vector< Vector2<int> > updatedPoints, GLDisplay::display displayId);
    void transitionSegmentsDisplayChanged(std::vector< TransitionSegment > updatedTransitionSegments, GLDisplay::display displayId);
    void robotCandidatesDisplayChanged(std::vector< RobotCandidate > updatedRobotCandidates, GLDisplay::display displayId);
    void lineDetectionDisplayChanged(std::vector<LSFittedLine> fieldLines, GLDisplay::display displayId);
    void candidatesDisplayChanged(std::vector< ObjectCandidate > updatedCandidates, GLDisplay::display displayId);

private:
    class classEntry
    {
        public:
        classEntry(): index(0), colour(0){}
        classEntry(int newIndex, unsigned char newColour): index(newIndex), colour(newColour){}
        int index;
        unsigned char colour;
    };


    void processVisionFrame(NUimage& image);
    void processVisionFrame(ClassifiedImage& image);

    void processVisionFrame();
    void generateClassifiedImage(const NUimage& yuvImage);

    // File Access
    NUbotImage* file;               //!< Instance of the File reader
    unsigned char* classificationTable;
    unsigned char* tempLut;


    // Image Storage
    NUimage rawImage;
    ClassifiedImage classImage, previewClassImage;
    Vision vision;

    Horizon horizonLine;
    //TODO: these should change later..
    float jointSensors[100];
    float balanceSensors[100];
    float touchSensors[100];
    static const int maxUndoLength = 10;
    int nextUndoIndex;
    bool hasImage;
    std::vector<classEntry> undoHistory[maxUndoLength];
};

#endif // VIRTUALNUBOT_H
