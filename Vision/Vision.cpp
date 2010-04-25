/*!
  @file Vision.h
  @brief Declaration of NUbots Vision class.
  @author Steven Nicklin
*/

#include "Vision.h"
#include "Tools/Image/NUimage.h"
#include "Tools/Math/Line.h"
#include "ClassificationColours.h"
#include "Ball.h"
#include "GoalDetection.h"
#include "Tools/Math/General.h"
#include <boost/circular_buffer.hpp>
#include <queue>
#include <algorithm>
#include "debug.h"
#include "debugverbosityvision.h"
#include "Tools/FileFormats/LUTTools.h"
#include "nubotdataconfig.h"

#include "NUPlatform/NUCamera.h"
#include "Behaviour/Jobs/JobList.h"
#include "Behaviour/Jobs/CameraJobs/ChangeCameraSettingsJob.h"
#include "Behaviour/Jobs/VisionJobs/SaveImagesJob.h"
#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"
#include "NUPlatform/NUActionators/NUSounds.h"
#include "NUPlatform/NUIO.h"

#include "Vision/Threads/SaveImagesThread.h"
#include <iostream>
//#include <QDebug>

using namespace mathGeneral;
Vision::Vision()
{

    AllFieldObjects = new FieldObjects();
    classifiedCounter = 0;
    LUTBuffer = new unsigned char[LUTTools::LUT_SIZE];
    currentLookupTable = LUTBuffer;
    loadLUTFromFile(string(DATA_DIR) + string("default.lut"));
    m_saveimages_thread = new SaveImagesThread(this);
    isSavingImages = false;
    ImageFrameNumber = 0;
    return;
}

Vision::~Vision()
{
    delete AllFieldObjects;
    delete [] LUTBuffer;
    return;
}

void Vision::process(JobList* jobs, NUCamera* camera, NUIO* m_io)
{
    //debug  << "Vision::Process - Begin" << endl;
    m_camera = camera;
    static list<Job*>::iterator it;     // the iterator over the motion jobs
    for (it = jobs->camera_begin(); it != jobs->camera_end();)
    {
        //debug  << "Vision::Process - Processing Job" << endl;
        if ((*it)->getID() == Job::CAMERA_CHANGE_SETTINGS)
        {   // process a walk speed job
            CameraSettings settings;
            static ChangeCameraSettingsJob* job;
            job = (ChangeCameraSettingsJob*) (*it);
            settings = job->getSettings();
            #if DEBUG_VISION_VERBOSITY > 4
                debug << "Vision::process(): Processing a camera job." << endl;
            #endif
            
            if( settings.exposure == -1 ||
                settings.contrast == -1 ||
                settings.gain     == -1 )
            {
                #if DEBUG_VISION_VERBOSITY > 4
                    debug << "Vision::Process - Send CAMERASETTINGS Request Recieved: " << endl;
                #endif
                JobList toSendList;
                ChangeCameraSettingsJob newJob(m_camera->getSettings());
                toSendList.addCameraJob(&newJob);
                (*m_io) << toSendList;
                toSendList.clear();
            }
            else
            {   
                m_camera->setSettings(settings);
            }
            it = jobs->removeCameraJob(it);
        }
        else 
        {
            ++it;
        }

    }
    
    for (it = jobs->vision_begin(); it != jobs->vision_end();)
    {
        if ((*it)->getID() == Job::VISION_SAVE_IMAGES)
        {   
            #if DEBUG_VISION_VERBOSITY > 4
                debug << "Vision::process(): Processing a save images job." << endl;
            #endif
            static SaveImagesJob* job;
            job = (SaveImagesJob*) (*it);
            if(isSavingImages != job->saving())
            {
                if(job->saving() == true)
                {
                    
                    currentSettings = m_camera->getSettings();
                    if (!imagefile.is_open())
                        imagefile.open((string(DATA_DIR) + string("images.nul")).c_str());
                    m_actions->addSound(m_sensor_data->CurrentTime, NUSounds::START_SAVING_IMAGES);
                }
                else
                {
                    
                    m_camera->setSettings(currentSettings);
                    m_actions->addSound(m_sensor_data->CurrentTime, NUSounds::STOP_SAVING_IMAGES);
                }
                isSavingImages = job->saving();
            }
            it = jobs->removeVisionJob(it);
         }
        else 
        {
            ++it;
        }

    }
}


FieldObjects* Vision::ProcessFrame(NUimage* image, NUSensorsData* data, NUActionatorsData* actions)
{
    #if DEBUG_VISION_VERBOSITY > 4
        debug << "Vision::ProcessFrame()." << endl;
    #endif

    if (image == NULL || data == NULL || actions == NULL)
        return AllFieldObjects;
    m_sensor_data = data;
    m_actions = actions;
    setImage(image);
    AllFieldObjects->preProcess(image->m_timestamp);

    std::vector< Vector2<int> > points;
    //std::vector< Vector2<int> > verticalPoints;
    std::vector< TransitionSegment > verticalsegments;
    std::vector< TransitionSegment > horizontalsegments;
    std::vector< TransitionSegment > allsegments;
    std::vector< TransitionSegment > segments;
    std::vector< ObjectCandidate > candidates;
    std::vector< ObjectCandidate > tempCandidates;
    //std::vector< Vector2<int> > horizontalPoints;
    //std::vector<LSFittedLine> fieldLines;
    int spacings = 16;
    Circle circ;
    int tempNumScanLines = 0;
    int robotClassifiedPoints = 0;
    //debug << "Setting Image: " <<endl;

    if(isSavingImages)
    {
        #if DEBUG_VISION_VERBOSITY > 1
            debug << "Vision::starting the save images loop." << endl;
        #endif
        m_saveimages_thread->startLoop();
    }
    //debug << "Generating Horizon Line: " <<endl;
    //Generate HorizonLine:
    vector <float> horizonInfo;
    Horizon horizonLine;

    if(m_sensor_data->getHorizon(horizonInfo))
    {
        horizonLine.setLine((double)horizonInfo[0],(double)horizonInfo[1],(double)horizonInfo[2]);
    }
    else
    {
        //debug << "No Horizon Data" << endl;
        return AllFieldObjects;
    }
    //debug << "Generating Horizon Line: Finnished" <<endl;
    //debug << "Image(0,0) is below: " << horizonLine.IsBelowHorizon(0, 0)<< endl;
    std::vector<unsigned char> validColours;
    Vision::tCLASSIFY_METHOD method;
    const int ROBOTS = 0;
    const int BALL   = 1;
    const int YELLOW_GOALS  = 2;
    const int BLUE_GOALS  = 3;
    int mode  = ROBOTS;


    //qDebug() << "CASE YUYVGenerate Classified Image: START";
    classifiedCounter = 0;
    //ClassifiedImage target(currentImage->getWidth(),currentImage->getHeight(),true);
    //classifyImage(target);

    //qDebug() << "Generate Classified Image: finnished";
    //setImage(&image);
    //! Find the green edges

    points = findGreenBorderPoints(spacings,&horizonLine);
    //emit pointsDisplayChanged(points,GLDisplay::greenHorizonScanPoints);
    //qDebug() << "Find Edges: finnished";
    //! Find the Field border
    points = getConvexFieldBorders(points);
    points = interpolateBorders(points,spacings);
    //debug << "Generating Green Boarder: Finnished" <<endl;
    //emit pointsDisplayChanged(points,GLDisplay::greenHorizonPoints);
    //qDebug() << "Find Field border: finnished";
    //! Scan Below Horizon Image
    ClassifiedSection vertScanArea = verticalScan(points,spacings);
    //debug << "Vert ScanPaths : Finnished " << vertScanArea.getNumberOfScanLines() <<endl;
    //! Scan Above the Horizon
    ClassifiedSection horiScanArea = horizontalScan(points,spacings);
    //debug << "Horizontal ScanPaths : Finnished " << horiScanArea.getNumberOfScanLines() <<endl;
    //qDebug() << "Generate Scanlines: finnished";
    
    //! Classify Line Segments
    ClassifyScanArea(&vertScanArea);
    ClassifyScanArea(&horiScanArea);
    //debug << "Classify ScanPaths : Finnished" <<endl;

    //! Extract and Display Vertical Scan Points:
    tempNumScanLines = vertScanArea.getNumberOfScanLines();
    for (int i = 0; i < tempNumScanLines; i++)
    {
        ScanLine* tempScanLine = vertScanArea.getScanLine(i);
        for(int seg = 0; seg < tempScanLine->getNumberOfSegments(); seg++)
        {
            verticalsegments.push_back((*tempScanLine->getSegment(seg)));
            segments.push_back((*tempScanLine->getSegment(seg)));
        }
    }

    //! Extract and Display Horizontal Scan Points:
    tempNumScanLines = horiScanArea.getNumberOfScanLines();
    for (int i = 0; i < tempNumScanLines; i++)
    {
        ScanLine* tempScanLine = horiScanArea.getScanLine(i);
        for(int seg = 0; seg < tempScanLine->getNumberOfSegments(); seg++)
        {
            horizontalsegments.push_back((*tempScanLine->getSegment(seg)));
            allsegments.push_back((*tempScanLine->getSegment(seg)));
        }
    }
    //! Form Lines
    //fieldLines = vision.DetectLines(vertScanArea,spacings);
    //! Extract Detected Line & Corners
    //emit lineDetectionDisplayChanged(fieldLines,GLDisplay::FieldLines);



    //emit pointsDisplayChanged(horizontalPoints,GLDisplay::horizontalScanPath);
    //emit pointsDisplayChanged(verticalPoints,GLDisplay::verticalScanPath);
    //qDebug() << "disaplay scanPaths: finnished";

    //emit transitionSegmentsDisplayChanged(allsegments,GLDisplay::TransitionSegments);

    //! Identify Field Objects

    //qDebug() << "PREclassifyCandidates";
    std::vector< ObjectCandidate > RobotCandidates;
    std::vector< ObjectCandidate > BallCandidates;
    std::vector< ObjectCandidate > BlueGoalCandidates;
    std::vector< ObjectCandidate > YellowGoalCandidates;
    std::vector< ObjectCandidate > BlueGoalAboveHorizonCandidates;
    std::vector< ObjectCandidate > YellowGoalAboveHorizonCandidates;

    mode = ROBOTS;
    method = Vision::PRIMS;
   for (int i = 0; i < 4; i++)
    {

        switch (i)
        {
            case ROBOTS:
                validColours.clear();
                validColours.push_back(ClassIndex::white);
                validColours.push_back(ClassIndex::red);
                validColours.push_back(ClassIndex::red_orange);
                validColours.push_back(ClassIndex::shadow_blue);
                //qDebug() << "PRE-ROBOT";

                RobotCandidates = classifyCandidates(verticalsegments, points, validColours, spacings, 0.2, 2.0, 12, method);
                //qDebug() << "POST-ROBOT";
                robotClassifiedPoints = 0;
                break;
            case BALL:
                validColours.clear();
                validColours.push_back(ClassIndex::orange);
                validColours.push_back(ClassIndex::red_orange);
                //validColours.push_back(ClassIndex::yellow_orange);
                //qDebug() << "PRE-BALL";
                BallCandidates = classifyCandidates(verticalsegments, points, validColours, spacings, 0, 3.0, 1, method);
                //qDebug() << "POST-BALL";
                break;
            case YELLOW_GOALS:
                validColours.clear();
                validColours.push_back(ClassIndex::yellow);
                validColours.push_back(ClassIndex::yellow_orange);
                //qDebug() << "PRE-GOALS";
                //tempCandidates = classifyCandidates(segments, points, validColours, spacings, 0.1, 4.0, 2, method);
                YellowGoalAboveHorizonCandidates = ClassifyCandidatesAboveTheHorizon(horizontalsegments,validColours,spacings,3);
                YellowGoalCandidates = classifyCandidates(verticalsegments, points, validColours, spacings, 0.1, 4.0, 2, method);
                //qDebug() << "POST-GOALS";
            case BLUE_GOALS:
                validColours.clear();
                validColours.push_back(ClassIndex::blue);
                validColours.push_back(ClassIndex::shadow_blue);
                //qDebug() << "PRE-GOALS";
                BlueGoalAboveHorizonCandidates = ClassifyCandidatesAboveTheHorizon(horizontalsegments,validColours,spacings,3);
                BlueGoalCandidates = classifyCandidates(verticalsegments, points, validColours, spacings, 0.1, 4.0, 2, method);
                //qDebug() << "POST-GOALS";
                break;
        }
    }
    if(BallCandidates.size() > 0)
    {
        circ = DetectBall(BallCandidates);
    }
    DetectGoals(YellowGoalCandidates, YellowGoalAboveHorizonCandidates, horizontalsegments);
    DetectGoals(BlueGoalCandidates, BlueGoalAboveHorizonCandidates, horizontalsegments);

    #if DEBUG_VISION_VERBOSITY > 4
	debug 	<< "Vision::ProcessFrame - Number of Pixels Classified: " << classifiedCounter 
			<< "\t Percent of Image: " << classifiedCounter / float(currentImage->getWidth() * currentImage->getHeight()) * 100.00 << "%" << endl;
    #endif
    
    AllFieldObjects->postProcess(image->m_timestamp);
    return AllFieldObjects;
}

void Vision::SaveAnImage()
{
    #if DEBUG_VISION_VERBOSITY > 1
        debug << "Vision::SaveAnImage(). Starting..." << endl;
    #endif
    //std::stringstream buffer;
    //buffer << *currentImage;
    NUimage buffer;
    buffer.cloneExisting(*currentImage);
    if (imagefile.is_open())
        imagefile << buffer;
    else
        debug << "Vision::SaveAnImage() Unable to saving images. Probably because the directory does not exist" << endl;

    //Set NextCameraSetting:
    CameraSettings tempCameraSettings = m_camera->getSettings();
    if(ImageFrameNumber % 6 == 0 )
    {
        tempCameraSettings.exposure = 100;
    }
    else if(ImageFrameNumber % 6 == 1 )
    {
        tempCameraSettings.exposure = 150;
    }
    else if( ImageFrameNumber % 6 == 2 )
    {
        tempCameraSettings.exposure = 200;
    }
    else if( ImageFrameNumber % 6 == 3 )
    {
        tempCameraSettings.exposure = 250;
    }
    else if( ImageFrameNumber % 6 == 4 )
    {
        tempCameraSettings.exposure = 300;
    }
    else if( ImageFrameNumber % 6 == 5 )
    {
        tempCameraSettings.exposure = 400;
    }
    m_camera->setSettings(tempCameraSettings);
    #if DEBUG_VISION_VERBOSITY > 1
        debug << "Vision::SaveAnImage(). Finished" << endl;
    #endif
}

void Vision::setLUT(unsigned char* newLUT)
{
    currentLookupTable = newLUT;
    return;
}

void Vision::loadLUTFromFile(const std::string& fileName)
{
    LUTTools lutLoader;
    if (lutLoader.LoadLUT(LUTBuffer, LUTTools::LUT_SIZE,fileName.c_str()) == true)
        setLUT(LUTBuffer);
    else
        errorlog << "Vision::loadLUTFromFile(" << fileName << "). Failed to load lut." << endl;
}

void Vision::setImage(const NUimage* newImage)
{
    currentImage = newImage;
    ImageFrameNumber++;
}

unsigned char Vision::classifyPixel(int x, int y)
{
    classifiedCounter++;
    Pixel* temp = &currentImage->m_image[y][x];
    //return  currentLookupTable[(temp->y<<16) + (temp->cb<<8) + temp->cr]; //8 bit LUT
    return  currentLookupTable[LUTTools::getLUTIndex(*temp)]; // 7bit LUT
    //return  currentLookupTable[((temp->y >> 2)<<12) + ((temp->cb >> 2)<<6) + (temp->cr>>2)]; // 6bit LUT
}
void Vision::classifyPreviewImage(ClassifiedImage &target,unsigned char* tempLut)
{
    //qDebug() << "InVision CLASS Generation:";
    int tempClassCounter = classifiedCounter;
    int width = currentImage->getWidth();
    int height = currentImage->getHeight();
    //qDebug() << sourceImage->width() << ","<< sourceImage->height();

    target.setImageDimensions(width,height);
    //qDebug() << "Set Dimensions:";
    //currentImage = sourceImage;
    const unsigned char * beforeLUT = currentLookupTable;
    currentLookupTable = tempLut;
    //qDebug() << "Begin Loop:";
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            target.image[y][x] = classifyPixel(x,y);
        }
    }
    classifiedCounter = tempClassCounter;
    currentLookupTable = beforeLUT;
    return;
}
void Vision::classifyImage(ClassifiedImage &target)
{
    //qDebug() << "InVision CLASS Generation:";
    int tempClassCounter = classifiedCounter;
    int width = currentImage->getWidth();
    int height = currentImage->getHeight();
    //qDebug() << sourceImage->width() << ","<< sourceImage->height();

    target.setImageDimensions(width,height);
    //qDebug() << "Set Dimensions:";
    //currentImage = sourceImage;
    //currentLookupTable = lookUpTable;
    //qDebug() << "Begin Loop:";
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            target.image[y][x] = classifyPixel(x,y);
        }
    }
    classifiedCounter = tempClassCounter;
    return;
}

std::vector< Vector2<int> > Vision::findGreenBorderPoints(int scanSpacing, Horizon* horizonLine)
{
    classifiedCounter = 0;
    std::vector< Vector2<int> > results;
    int width = currentImage->getWidth();
    int height = currentImage->getHeight();
    //debug << "Finding Green Boarders: "  << scanSpacing << "  Under Horizon: " << horizonLine->getA() << "x + " << horizonLine->getB() << "y + " << horizonLine->getC() << " = 0" << endl;
    //debug << width << " , "<< height << endl;
    int yStart;
    int consecutiveGreenPixels = 0;
    for (int x = 0; x < width; x+=scanSpacing)
    {
        yStart = (int)horizonLine->findYFromX(x);
        if(yStart > height) continue;
        if(yStart < 0) yStart = 0;
        consecutiveGreenPixels = 0;
        for (int y = yStart; y < height; y++)
        {
            if(classifyPixel(x,y) == ClassIndex::green)
            {
                consecutiveGreenPixels++;
            }
            else
            {
                consecutiveGreenPixels = 0;
            }
            if(consecutiveGreenPixels >= 6)
            {
                results.push_back(Vector2<int>(x,y-consecutiveGreenPixels+1));
                break;
            }
        }
    }
    return results;
}

#define LEFT_OF(x0, x1, x2) ((x1.x-x0.x)*(-x2.y+x0.y)-(x2.x-x0.x)*(-x1.y+x0.y) > 0)

std::vector<Vector2<int> > Vision::getConvexFieldBorders(std::vector<Vector2<int> >& fieldBorders)
{
  //Andrew's Monotone Chain Algorithm to compute the upper hull
  std::vector<Vector2<int> > hull;
  if(!fieldBorders.size()) return hull;
  const std::vector<Vector2<int> >::const_iterator pmin = fieldBorders.begin(),
                                                   pmax = fieldBorders.end()-1;
  hull.push_back(*pmin);
  for(std::vector<Vector2<int> >::const_iterator pi = pmin + 1; pi != pmax+1; pi++)
  {
    if(!LEFT_OF((*pmin), (*pmax), (*pi)) && pi != pmax)
      continue;

    while((int)hull.size() > 1)
    {
      const std::vector<Vector2<int> >::const_iterator p1 = hull.end() - 1,
                                                       p2 = hull.end() - 2;
      if(LEFT_OF((*p1), (*p2), (*pi)))
        break;
      hull.pop_back();
    }
    hull.push_back(*pi);
  }
  return hull;
}

std::vector<Vector2<int> > Vision::interpolateBorders(std::vector<Vector2<int> >& fieldBorders, int scanSpacing)
{
    std::vector<Vector2<int> > interpolatedBorders;
    if(!fieldBorders.size()) return interpolatedBorders;
    std::vector<Vector2<int> >::const_iterator nextPoint = fieldBorders.begin();
    std::vector<Vector2<int> >::const_iterator prevPoint = nextPoint++;

    int height = currentImage->getHeight();

    int x = prevPoint->x;
    Vector2<int> deltaPoint, temp;
    for (; nextPoint != fieldBorders.end(); nextPoint++)
    {
        deltaPoint = (*nextPoint) - (*prevPoint);
        for (; x <= nextPoint->x; x+=scanSpacing)
        {
            temp.x = x;
            temp.y = (x - prevPoint->x) * deltaPoint.y / deltaPoint.x + prevPoint->y;
            if (temp.y < 0) temp.y = 0;
            if (temp.y >= height) temp.y = height - 1;
            interpolatedBorders.push_back(temp);
        }
        prevPoint = nextPoint;
    }
    return interpolatedBorders;
}

ClassifiedSection Vision::verticalScan(std::vector<Vector2<int> >&fieldBorders,int scanSpacing)
{
    //std::vector<Vector2<int> > scanPoints;
    ClassifiedSection scanArea(ClassifiedSection::DOWN);
    if(!fieldBorders.size()) return scanArea;
    std::vector<Vector2<int> >::const_iterator nextPoint = fieldBorders.begin();
    //std::vector<Vector2<int> >::const_iterator prevPoint = nextPoint++; //This iterator is unused
    int x = 0;
    int y = 0;
    int fullLineLength = 0;
    int halfLineLength = 0;
    int quarterLineLength = 0;
    int midX = 0;
    int skip = int(scanSpacing/2);

    int height = currentImage->getHeight();

    Vector2<int> temp;
    for (; nextPoint != fieldBorders.end(); nextPoint++)
    {
        x = nextPoint->x;
        y = nextPoint->y;

        //!Create Full ScanLine
        temp.x = x;
        temp.y = y;

        fullLineLength = int(height - y);
        ScanLine tempScanLine(temp, fullLineLength);
        scanArea.addScanLine(tempScanLine);

        //!Create half ScanLine
        midX = x-skip;
        temp.x = midX;
        halfLineLength = int((height - y)/2);
        ScanLine tempMidScanLine(temp,halfLineLength);
        scanArea.addScanLine(tempMidScanLine);

        //!Create Quarter ScanLines
        temp.x = int(midX - skip/2);
        quarterLineLength = int((height - y)/4);
        ScanLine tempLeftQuarterLine(temp,quarterLineLength);
        scanArea.addScanLine(tempLeftQuarterLine);
        temp.x = int(midX + skip/2);
        ScanLine tempRightQuarterLine(temp,quarterLineLength);
        scanArea.addScanLine(tempRightQuarterLine);
    }

    //!Generate the last Lines:
    midX = fieldBorders.back().x+skip;
    y = fieldBorders.back().y;
    temp.x = midX;
    temp.y = y;
    ScanLine tempMidScanLine(temp,halfLineLength);
    scanArea.addScanLine(tempMidScanLine);
    temp.x = midX-skip/2;
    temp.y = y;
    ScanLine tempLeftQuarterLine(temp,quarterLineLength);
    scanArea.addScanLine(tempLeftQuarterLine);
    temp.x = midX+skip/2;
    temp.y = y;
    ScanLine tempRightQuarterLine(temp,quarterLineLength);
    scanArea.addScanLine(tempRightQuarterLine);

    return scanArea;
}

ClassifiedSection Vision::horizontalScan(std::vector<Vector2<int> >&fieldBorders,int scanSpacing)
{
    ClassifiedSection scanArea(ClassifiedSection::RIGHT);
    if(!currentImage) return scanArea;
    Vector2<int> temp;
    int width = currentImage->getWidth();
    int height = currentImage->getHeight();
    //! Case for No FieldBorders
    if(!fieldBorders.size())
    {

        for(int y = 0; y < height; y = y + scanSpacing*2)
        {
            temp.x = 0;
            temp.y = y;
            ScanLine tempScanLine(temp,width);
            scanArea.addScanLine(tempScanLine);
        }
        return scanArea;
    }

    //! Find the minimum Y, and scan above the field boarders

    std::vector<Vector2<int> >::const_iterator nextPoint = fieldBorders.begin();
    std::vector<Vector2<int> >::const_iterator prevPoint = nextPoint++;
    int minY = height;
    int minX = width;
    int maxY = 0;
    int maxX = 0;
    for (; nextPoint != fieldBorders.end(); nextPoint++)
    {
        if(nextPoint->y < minY)
        {
            minY = nextPoint->y;
        }
        if(nextPoint->y >maxY)
        {
            maxY = nextPoint->y;
        }
        if(nextPoint->x < minX)
        {
            minX = nextPoint->x;
        }
        if(nextPoint->x > maxX)
        {
            maxX = nextPoint->x;
        }
    }

    //! Then calculate horizontal scanlines above the field boarder
    //! Generate Scan pattern for above the max of green boarder.
    for(int y = 0; y < minY; y = y + scanSpacing)
    {
        temp.x =0;
        temp.y = y;
        ScanLine tempScanLine(temp,width);
        scanArea.addScanLine(tempScanLine);
    }
    //! Generate Scan Pattern for in between the max and min of green horizon.
    for(int y = minY; y < maxY; y = y + scanSpacing*2)
    {
        temp.x =0;
        temp.y = y;
        ScanLine tempScanLine(temp,width);
        scanArea.addScanLine(tempScanLine);
    }
    //! Generate Scan Pattern premature end of horizon
    for(int y = maxY; y < height; y = y + scanSpacing*2)
    {
        if(maxX > width - scanSpacing*3)
        {
            break;
        }
        temp.x = maxX;
        temp.y = y;
        ScanLine tempScanLine(temp,width-maxX);
        scanArea.addScanLine(tempScanLine);
    }
    for(int y = maxY; y < height; y = y + scanSpacing*2)
    {
        if(minX < scanSpacing*3)
        {
            break;
        }
        temp.x = 0;
        temp.y = y;
        ScanLine tempScanLine(temp,minX);
        scanArea.addScanLine(tempScanLine);
    }
    return scanArea;
}

void Vision::ClassifyScanArea(ClassifiedSection* scanArea)
{
    int direction = scanArea->getDirection();
    int numOfLines = scanArea->getNumberOfScanLines();
    int lineLength = 0;
    int skipPixel = 2;
    ScanLine* tempLine;
    Vector2<int> currentPoint;
    Vector2<int> tempStartPoint;
    Vector2<int> startPoint;
    unsigned char beforeColour = 0; //!< Colour Before the segment
    unsigned char afterColour = 0;  //!< Colour in the next Segment
    unsigned char currentColour = 0; //!< Colour in the current segment
    //! initialising circular buffer
    int bufferSize = 2;
    boost::circular_buffer<unsigned char> colourBuff(bufferSize);
    for (int i = 0; i < bufferSize; i++)
    {
        colourBuff.push_back(0);
    }
    for (int i = 0; i < numOfLines; i++)
    {
        tempLine = scanArea->getScanLine(i);
        startPoint = tempLine->getStart();
        lineLength = tempLine->getLength();
        tempStartPoint = startPoint;


        beforeColour    = 0; //!< Colour Before the segment
        afterColour     = 0;  //!< Colour in the next Segment
        currentColour   = 0; //!< Colour in the current segment

        //! No point in scanning lines less then the buffer size
        if(lineLength < bufferSize) continue;

        for(int j = 0; j < lineLength; j = j+skipPixel)
        {
            if(direction == ClassifiedSection::DOWN)
            {
                currentPoint.x = startPoint.x;
                currentPoint.y = startPoint.y + j;
            }
            else if (direction == ClassifiedSection::RIGHT)
            {
                currentPoint.x = startPoint.x + j;
                currentPoint.y = startPoint.y;
            }
            else if(direction == ClassifiedSection::UP)
            {
                currentPoint.x = startPoint.x;
                currentPoint.y = startPoint.y - j;
            }
            else if(direction == ClassifiedSection::LEFT)
            {
                currentPoint.x = startPoint.x - j;
                currentPoint.y = startPoint.y;
            }
            //debug << currentPoint.x << " " << currentPoint.y;
            afterColour = classifyPixel(currentPoint.x,currentPoint.y);
            colourBuff.push_back(afterColour);

            if(j >= lineLength - skipPixel)
            {
                //! End Of SCANLINE detected: Continue scnaning and when buffer ends or end of screen Generate new segment and add to the line
                if((currentColour == ClassIndex::green || currentColour == ClassIndex::unclassified || currentColour == ClassIndex::shadow_object))
                {
                    tempStartPoint = currentPoint;
                    beforeColour = ClassIndex::unclassified;
                    currentColour = afterColour;
                    for (int i = 0; i < bufferSize; i++)
                    {
                        colourBuff.push_back(0);
                    }
                    continue;
                }

                while( ( checkIfBufferSame(colourBuff) && currentColour == afterColour) )
                {

                    if(direction == ClassifiedSection::DOWN)
                    {

                        if(startPoint.y + j < currentImage->getHeight())
                        {
                            currentPoint.y = startPoint.y + j;
                            currentPoint.x = startPoint.x;
                        }
                        else
                        {
                            break;
                        }
                    }
                    else if (direction == ClassifiedSection::RIGHT)
                    {
                        if(startPoint.x + j < currentImage->getWidth())
                        {
                            currentPoint.x = startPoint.x + j;
                            currentPoint.y = startPoint.y;
                        }
                        else
                        {
                            break;
                        }

                    }
                    else if(direction == ClassifiedSection::UP)
                    {

                        if(startPoint.y - j > 0)
                        {
                            currentPoint.y = startPoint.y - j;
                            currentPoint.x = startPoint.x;
                        }
                        else
                        {
                            break;
                        }

                    }
                    else if(direction == ClassifiedSection::LEFT)
                    {
                        if(startPoint.x - j > 0)
                        {
                            currentPoint.x = startPoint.x - j;
                            currentPoint.y = startPoint.y;
                        }
                        else
                        {
                            break;
                        }
                    }
                    afterColour = classifyPixel(currentPoint.x,currentPoint.y);
                    colourBuff.push_back(afterColour);
                    j = j+skipPixel*3;
                    /*qDebug() << "Scanning: " << skipPixel<<","<<j << "\t"<< currentPoint.x << "," << currentPoint.y <<
                            "\t"<<currentColour<< "," << afterColour <<
                            "\t"<< currentPoint.y+j << "," << currentImage->getHeight() <<
                            "\t"<< currentPoint.x+j << "," << currentImage->getWidth();*/
                }

                TransitionSegment tempTransition(tempStartPoint, currentPoint, beforeColour, currentColour, afterColour);
                tempLine->addSegement(tempTransition);

                tempStartPoint = currentPoint;
                beforeColour = ClassIndex::unclassified;
                currentColour = afterColour;
                for (int i = 0; i < bufferSize; i++)
                {
                    colourBuff.push_back(0);
                }
                continue;
            }

            if(checkIfBufferSame(colourBuff))
            {
                if(currentColour != afterColour)
                {
                    //! Transition detected: Generate new segment and add to the line
                    //Adjust the position:
                    if(!(currentColour == ClassIndex::green || currentColour == ClassIndex::unclassified || currentColour == ClassIndex::shadow_object))
                    {
                        //SHIFTING THE POINTS TO THE START OF BUFFER:
                        if(direction == ClassifiedSection::DOWN)
                        {
                            currentPoint.x = startPoint.x;
                            currentPoint.y = startPoint.y + j - bufferSize * skipPixel/2;
                            if(tempStartPoint.y > startPoint.y)
                            {
                                tempStartPoint.y = tempStartPoint.y - bufferSize * skipPixel/2;
                            }
                        }
                        else if (direction == ClassifiedSection::RIGHT)
                        {
                            currentPoint.x = startPoint.x + j - bufferSize * skipPixel/2;
                            currentPoint.y = startPoint.y;
                            if(tempStartPoint.x > startPoint.x)
                            {
                                tempStartPoint.x = tempStartPoint.x - bufferSize * skipPixel/2;
                            }
                        }
                        else if(direction == ClassifiedSection::UP)
                        {
                            currentPoint.x = startPoint.x;
                            currentPoint.y = startPoint.y - j + bufferSize * skipPixel/2;
                            if(tempStartPoint.y < startPoint.y)
                            {
                                tempStartPoint.y = tempStartPoint.y + bufferSize * skipPixel/2;
                            }
                        }
                        else if(direction == ClassifiedSection::LEFT)
                        {
                            currentPoint.x = startPoint.x - j + bufferSize * skipPixel/2;
                            currentPoint.y = startPoint.y;
                            if(tempStartPoint.x < startPoint.x)
                            {
                                tempStartPoint.x = tempStartPoint.x + bufferSize * skipPixel/2;
                            }
                        }
                        TransitionSegment tempTransition(tempStartPoint, currentPoint, beforeColour, currentColour, afterColour);
                        tempLine->addSegement(tempTransition);
                    }
                    tempStartPoint = currentPoint;
                    beforeColour = currentColour;
                    currentColour = afterColour;
                    for (int i = 0; i < bufferSize; i++)
                    {
                        colourBuff.push_back(0);
                    }
                }
            }
        }

    }
    return;
}

void Vision::CloselyClassifyScanline(ScanLine* tempLine, TransitionSegment* tempTransition,int spacings, int direction)// Vector2<int> tempStartPoint, unsigned char currentColour, int length, int spacings, int direction)
{
    int width = currentImage->getWidth();
    int height = currentImage->getHeight();
    if((direction == ClassifiedSection::DOWN || direction == ClassifiedSection::UP))
    {
        Vector2<int> StartPoint = tempTransition->getStartPoint();
        int bufferSize = 10;
        boost::circular_buffer<unsigned char> colourBuff(bufferSize);
        for (int i = 0; i < bufferSize; i++)
        {
            colourBuff.push_back(tempTransition->getColour());
        }

        int length = abs(tempTransition->getEndPoint().y - tempTransition->getStartPoint().y);
        Vector2<int> tempSubEndPoint;
        Vector2<int> tempSubStartPoint;
        unsigned char subAfterColour;
        unsigned char subBeforeColour;
        unsigned char tempColour = tempTransition->getColour();

        for(int k = 0; k < length; k = k+spacings)
        {
            tempSubEndPoint.y   = StartPoint.y+k;
            tempSubStartPoint.y = StartPoint.y+k;
            int tempsubPoint    = StartPoint.x;
            tempColour          = tempTransition->getColour();
            //Reset Buffer: to OriginalColour
            for (int i = 0; i < bufferSize; i++)
            {
                colourBuff.push_back(tempTransition->getColour());
            }
            while(checkIfBufferSame(colourBuff))
            {
                if(tempsubPoint+1 > width)
                {
                    break;
                }

                tempsubPoint++;

                if(StartPoint.y+k < height && StartPoint.y+k > 0
                   && tempsubPoint < width && tempsubPoint > 0)
                {

                    tempColour= classifyPixel(tempsubPoint,StartPoint.y+k);
                    colourBuff.push_back(tempColour);
                }
                else
                {
                    break;
                }
            }

            tempSubEndPoint.x = tempsubPoint;
            subAfterColour = tempColour;
            tempsubPoint = StartPoint.x;
            tempColour = tempTransition->getColour();
            //Reset Buffer: to OriginalColour
            for (int i = 0; i < bufferSize; i++)
            {
                colourBuff.push_back(tempTransition->getColour());
            }
            while(checkIfBufferSame(colourBuff))
            {
                if(tempsubPoint-1 < 0) break;
                tempsubPoint--;
                if(StartPoint.y+k < height && StartPoint.y+k > 0
                   && tempsubPoint < width && tempsubPoint > 0)
                {
                    tempColour = classifyPixel(tempsubPoint,StartPoint.y+k);
                    colourBuff.push_back(tempColour);
                }
                else
                {
                    break;
                }
            }
            tempSubStartPoint.x = tempsubPoint;
            subBeforeColour = tempTransition->getColour();
            //THEN ADD TO LINE

            TransitionSegment tempTransitionA(tempSubStartPoint, tempSubEndPoint, subBeforeColour , tempTransition->getColour(), subAfterColour);
            if(tempTransitionA.getSize() >1)
            {
                tempLine->addSegement(tempTransitionA);
            }
        }
    }

    else if (direction == ClassifiedSection::RIGHT || direction == ClassifiedSection::LEFT)
    {
        Vector2<int> StartPoint = tempTransition->getStartPoint();

        int bufferSize = 10;
        boost::circular_buffer<unsigned char> colourBuff(bufferSize);


        int length = abs(tempTransition->getEndPoint().x - tempTransition->getStartPoint().x);
        Vector2<int> tempSubEndPoint;
        Vector2<int> tempSubStartPoint;
        unsigned char subAfterColour;
        unsigned char subBeforeColour;
        unsigned char tempColour = tempTransition->getColour();
        for(int k = 0; k < length; k = k+spacings)
        {
            tempSubEndPoint.x = StartPoint.x+k;
            tempSubStartPoint.x = StartPoint.x+k;
            int tempY = StartPoint.y;
            tempColour = tempTransition->getColour();
            //Reseting ColourBuffer
            for (int i = 0; i < bufferSize; i++)
            {
                colourBuff.push_back(tempTransition->getColour());
            }
            //Search for End of Perpendicular Segment
            while(checkIfBufferSame(colourBuff))
            {
                if(tempY+1 > height) break;
                tempY++;

                if(StartPoint.x+k < width && StartPoint.x+k > 0 &&
                   tempY < height && tempY > 0)
                {
                    tempColour= classifyPixel(StartPoint.x+k,tempY);
                    colourBuff.push_back(tempColour);
                }
                else
                {
                    break;
                }
            }

            tempSubEndPoint.y = tempY;
            subAfterColour = tempColour;
            tempY = StartPoint.y;
            tempColour = tempTransition->getColour();
            //Reseting ColourBuffer
            for (int i = 0; i < bufferSize; i++)
            {
                colourBuff.push_back(tempTransition->getColour());
            }

            //Search for Start of Perpendicular Segment
            while(checkIfBufferSame(colourBuff))
            {
                if(tempY-1 < 0)
                {
                    break;
                }
                tempY--;
                if(StartPoint.x+k < width && StartPoint.x+k > 0
                   && tempY < height && tempY > 0)
                {
                    tempColour = classifyPixel(StartPoint.x+k,tempY);
                    //debug << tempY<< "," << (int)tempColour<< endl;
                    colourBuff.push_back(tempColour);
                }
                else
                {
                    break;
                }
            }
            tempSubStartPoint.y = tempY;
            subBeforeColour = tempTransition->getColour();
            //THEN ADD TO LINE


            TransitionSegment tempTransitionA(tempSubStartPoint, tempSubEndPoint, subBeforeColour , tempTransition->getColour(), subAfterColour);
            if(tempTransitionA.getSize() >1)
            {
                tempLine->addSegement(tempTransitionA);
            }
        }
    }
}

std::vector<ObjectCandidate> Vision::classifyCandidates(
                                        std::vector< TransitionSegment > segments,
                                        std::vector<Vector2<int> >&fieldBorders,
                                        std::vector<unsigned char> validColours,
                                        int spacing,
                                        float min_aspect, float max_aspect, int min_segments,
                                        tCLASSIFY_METHOD method)
{
    switch(method)
    {
        case PRIMS:
            return classifyCandidatesPrims(segments, fieldBorders, validColours, spacing, min_aspect, max_aspect, min_segments);
        break;
        case DBSCAN:
            return classifyCandidatesDBSCAN(segments, fieldBorders, validColours, spacing, min_aspect, max_aspect, min_segments);
        break;
        default:
            return classifyCandidatesPrims(segments, fieldBorders, validColours, spacing,  min_aspect, max_aspect, min_segments);
        break;
    }

}

std::vector<ObjectCandidate> Vision::classifyCandidatesPrims(std::vector< TransitionSegment > segments,
                                        std::vector<Vector2<int> >&fieldBorders,
                                        std::vector<unsigned char> validColours,
                                        int spacing,
                                        float min_aspect, float max_aspect, int min_segments)
{
    //! Overall runtime O( (K*(2*M^3 + M^2) + N*(LogN + 1) )
    std::vector<ObjectCandidate> candidateList;

    const int VERT_JOIN_LIMIT = 3;
    const int HORZ_JOIN_LIMIT = 1;


    if (!segments.empty())
    {
        //! Sorting O(N*logN)
        sort(segments.begin(), segments.end(), Vision::sortTransitionSegments);

        std::queue<int> qUnprocessed;
        std::vector<TransitionSegment> candidate_segments;
        unsigned int rawSegsLeft = segments.size();
        unsigned int nextRawSeg = 0;

        bool isSegUsed [segments.size()];

        //! Removing invalid colours O(N)
        for (unsigned int i = 0; i < segments.size(); i++)
        {
            //may have non-robot colour segments, so pre-mark them as used
            if (isValidColour(segments.at(i).getColour(), validColours))
            {
                //qDebug() << ClassIndex::getColourNameFromIndex(segments.at(i).getColour()) << isRobotColour(segments.at(i).getColour());
                isSegUsed[i] = false;
                //qDebug() <<  "(" << segments.at(i).getStartPoint().x << "," << segments.at(i).getStartPoint().y << ")-("<< segments.at(i).getEndPoint().x << "," << segments.at(i).getEndPoint().y << ")[" << ClassIndex::getColourNameFromIndex(segments.at(i).getColour()) << "]";
            }
            else
            {
                rawSegsLeft--;
                isSegUsed[i] = true;
            }

        }

        //! For all valid segments O(M)
        while(rawSegsLeft)
        {

            //Roll through and find first unused segment
            nextRawSeg = 0;

            //! Find next unused segment O(M)
            while(isSegUsed[nextRawSeg] && nextRawSeg < segments.size()) nextRawSeg++;
            //Prime unprocessed segment queue to build next candidate
            qUnprocessed.push(nextRawSeg);
            //take away from pool of raw segs
            isSegUsed[nextRawSeg] = true;
            rawSegsLeft--;

            int min_x, max_x, min_y, max_y, segCount;
            int * colourHistogram = new int[validColours.size()];
            min_x = segments.at(nextRawSeg).getStartPoint().x;
            max_x = segments.at(nextRawSeg).getStartPoint().x;
            min_y = segments.at(nextRawSeg).getStartPoint().y;
            max_y = segments.at(nextRawSeg).getEndPoint().y;
            segCount = 0;
            for (unsigned int i = 0; i < validColours.size(); i++)  colourHistogram[i] = 0;

            //! For all unprocessed joined segment in a candidate O(M)
            //Build candidate

            candidate_segments.clear();

            while (!qUnprocessed.empty())
            {
                unsigned int thisSeg;
                thisSeg = qUnprocessed.front();
                qUnprocessed.pop();
                segCount++;
                for (unsigned int i = 0; i < validColours.size(); i++)
                {
                    if ( segments.at(thisSeg).getColour() == validColours.at(i) && validColours.at(i) != ClassIndex::white)
                    {
                        colourHistogram[i] += 1;
                        i = validColours.size();
                    }
                }

                if ( min_x > segments.at(thisSeg).getStartPoint().x)
                    min_x = segments.at(thisSeg).getStartPoint().x;
                if ( max_x < segments.at(thisSeg).getStartPoint().x)
                    max_x = segments.at(thisSeg).getStartPoint().x;
                if ( min_y > segments.at(thisSeg).getStartPoint().y)
                    min_y = segments.at(thisSeg).getStartPoint().y;
                if ( max_y < segments.at(thisSeg).getEndPoint().y)
                    max_y = segments.at(thisSeg).getEndPoint().y;



                //if there is a seg above AND 'close enough', then qUnprocessed->push()
                if ( thisSeg > 0 &&
                     !isSegUsed[thisSeg-1] &&
                     segments.at(thisSeg).getStartPoint().x == segments.at(thisSeg-1).getStartPoint().x &&
                     segments.at(thisSeg).getStartPoint().y - segments.at(thisSeg-1).getEndPoint().y < VERT_JOIN_LIMIT)
                {
                    //qDebug() << "Up   Join Seg: " << thisSeg << "(" << ClassIndex::getColourNameFromIndex(segments.at(thisSeg).getColour()) << ") U " << (thisSeg-1)<< "(" << ClassIndex::getColourNameFromIndex(segments.at(thisSeg-1).getColour()) << ")" << "(" << segments.at(thisSeg).getStartPoint().x << "," << segments.at(thisSeg).getStartPoint().y << ")-("<< segments.at(thisSeg).getEndPoint().x << "," << segments.at(thisSeg).getEndPoint().y << ")::(" << segments.at(thisSeg-1).getStartPoint().x << "," << segments.at(thisSeg-1).getStartPoint().y << ")-("<< segments.at(thisSeg-1).getEndPoint().x << "," << segments.at(thisSeg-1).getEndPoint().y << ")";
                    qUnprocessed.push(thisSeg-1);
                    //take away from pool of raw segs
                    isSegUsed[thisSeg-1] = true;
                    rawSegsLeft--;
                }

                //if there is a seg below AND 'close enough', then qUnprocessed->push()
                if ( thisSeg+1 < segments.size() &&
                     !isSegUsed[thisSeg+1] &&
                     segments.at(thisSeg).getStartPoint().x == segments.at(thisSeg+1).getStartPoint().x &&
                     segments.at(thisSeg+1).getStartPoint().y - segments.at(thisSeg).getEndPoint().y < VERT_JOIN_LIMIT)
                {
                    //qDebug() << "Down Join Seg: " << thisSeg << "(" << ClassIndex::getColourNameFromIndex(segments.at(thisSeg).getColour()) << ") U " << (thisSeg+1)<< "(" << ClassIndex::getColourNameFromIndex(segments.at(thisSeg+1).getColour()) << ")" << "(" << segments.at(thisSeg).getStartPoint().x << "," << segments.at(thisSeg).getStartPoint().y << ")-("<< segments.at(thisSeg).getEndPoint().x << "," << segments.at(thisSeg).getEndPoint().y << ")::(" << segments.at(thisSeg+1).getStartPoint().x << "," << segments.at(thisSeg+1).getStartPoint().y << ")-("<< segments.at(thisSeg+1).getEndPoint().x << "," << segments.at(thisSeg+1).getEndPoint().y << ")";
                    qUnprocessed.push(thisSeg+1);
                    //take away from pool of raw segs
                    isSegUsed[thisSeg+1] = true;
                    rawSegsLeft--;
                }

                //! For each segment being processed in a candidate to the RIGHT attempt to join segments within range O(M)
                //if there is a seg overlapping on the right AND 'close enough', then qUnprocessed->push()
                for (unsigned int thatSeg = thisSeg + 1; thatSeg < segments.size(); thatSeg++)
                {
                    if ( segments.at(thatSeg).getStartPoint().x - segments.at(thisSeg).getStartPoint().x <=  spacing*HORZ_JOIN_LIMIT)
                    {
                        if ( segments.at(thatSeg).getStartPoint().x > segments.at(thisSeg).getStartPoint().x &&
                             !isSegUsed[thatSeg])
                        {
                            //NOT in same column as thisSeg and is to the right
                            if ( segments.at(thatSeg).getStartPoint().y <= segments.at(thisSeg).getEndPoint().y &&
                                 segments.at(thisSeg).getStartPoint().y <= segments.at(thatSeg).getEndPoint().y)
                            {
                                //thisSeg overlaps with thatSeg
                                //qDebug() <<  "(" << segments.at(thisSeg).getStartPoint().x << "," << segments.at(thisSeg).getStartPoint().y << ")-("<< segments.at(thisSeg).getEndPoint().x << "," << segments.at(thisSeg).getEndPoint().y << ")[" << ClassIndex::getColourNameFromIndex(segments.at(thisSeg).getColour()) << "]::(" << segments.at(thatSeg).getStartPoint().x << "," << segments.at(thatSeg).getStartPoint().y << ")-("<< segments.at(thatSeg).getEndPoint().x << "," << segments.at(thatSeg).getEndPoint().y << ")[" << ClassIndex::getColourNameFromIndex(segments.at(thatSeg).getColour()) << "]";
                                //! Find intercept O(K), K is number of field border points
                                int intercept = findInterceptFromPerspectiveFrustum(fieldBorders,
                                                                                    segments.at(thisSeg).getStartPoint().x,
                                                                                    segments.at(thatSeg).getStartPoint().x,
                                                                                    spacing*HORZ_JOIN_LIMIT);
                                if ( intercept >= 0 &&
                                     segments.at(thatSeg).getEndPoint().y >= intercept &&
                                     intercept <= segments.at(thisSeg).getEndPoint().y)
                                {
                                    //within HORZ_JOIN_LIMIT
                                    //qDebug() << "Right Join Seg: " << thisSeg << "(" << ClassIndex::getColourNameFromIndex(segments.at(thisSeg).getColour()) << ") U " << (thatSeg)<< "(" << ClassIndex::getColourNameFromIndex(segments.at(thatSeg).getColour()) << ")" << "(" << segments.at(thisSeg).getStartPoint().x << "," << segments.at(thisSeg).getStartPoint().y << ")-("<< segments.at(thisSeg).getEndPoint().x << "," << segments.at(thisSeg).getEndPoint().y << ")::(" << segments.at(thatSeg).getStartPoint().x << "," << segments.at(thatSeg).getStartPoint().y << ")-("<< segments.at(thatSeg).getEndPoint().x << "," << segments.at(thatSeg).getEndPoint().y << ")";
                                    qUnprocessed.push(thatSeg);
                                    //take away from pool of raw segs
                                    isSegUsed[thatSeg] = true;
                                    rawSegsLeft--;

                                }
                                else
                                {
                                    //qDebug() << "|" << float(segments.at(thatSeg).getEndPoint().y) <<  "thatSeg End(y)>= intercept" << intercept << "|";
                                    //qDebug() << "|" << int(intercept) << "intercept <= thisSeg End(y)" << segments.at(thisSeg).getEndPoint().y << "|";
                                }
                            }
                        }
                    }
                    else
                    {
                        thatSeg = segments.size();
                    }
                }

                //! For each segment being processed in a candidate to the LEFT attempt to join segments within range O(M)
                //if there is a seg overlapping on the left AND 'close enough', then qUnprocessed->push()
                for (int thatSeg = thisSeg - 1; thatSeg >= 0; thatSeg--)
                {
                    if ( segments.at(thisSeg).getStartPoint().x - segments.at(thatSeg).getStartPoint().x <=  spacing*HORZ_JOIN_LIMIT)
                    {

                        if ( !isSegUsed[thatSeg] &&
                             segments.at(thatSeg).getStartPoint().x < segments.at(thisSeg).getStartPoint().x)
                        {
                            //NOT in same column as thisSeg and is to the right
                            if ( segments.at(thatSeg).getStartPoint().y <= segments.at(thisSeg).getEndPoint().y &&
                                 segments.at(thisSeg).getStartPoint().y <= segments.at(thatSeg).getEndPoint().y)
                            {
                                //thisSeg overlaps with thatSeg
                                //qDebug() <<  "(" << segments.at(thisSeg).getStartPoint().x << "," << segments.at(thisSeg).getStartPoint().y << ")-("<< segments.at(thisSeg).getEndPoint().x << "," << segments.at(thisSeg).getEndPoint().y << ")[" << ClassIndex::getColourNameFromIndex(segments.at(thisSeg).getColour()) << "]::(" << segments.at(thatSeg).getStartPoint().x << "," << segments.at(thatSeg).getStartPoint().y << ")-("<< segments.at(thatSeg).getEndPoint().x << "," << segments.at(thatSeg).getEndPoint().y << ")[" << ClassIndex::getColourNameFromIndex(segments.at(thatSeg).getColour()) << "]";
                                //! Find intercept O(K), K is number of field border points
                                int intercept = findInterceptFromPerspectiveFrustum(fieldBorders,
                                                                                    segments.at(thisSeg).getStartPoint().x,
                                                                                    segments.at(thatSeg).getStartPoint().x,
                                                                                    spacing*HORZ_JOIN_LIMIT);

                                if ( intercept >= 0 &&
                                     segments.at(thatSeg).getEndPoint().y >= intercept &&
                                     intercept <= segments.at(thisSeg).getEndPoint().y)
                                {
                                    //within HORZ_JOIN_LIMIT
                                    //qDebug() << "Left Join Seg: " << thisSeg << "(" << ClassIndex::getColourNameFromIndex(segments.at(thisSeg).getColour()) << ") U " << (thatSeg)<< "(" << ClassIndex::getColourNameFromIndex(segments.at(thatSeg).getColour()) << ")" << "(" << segments.at(thisSeg).getStartPoint().x << "," << segments.at(thisSeg).getStartPoint().y << ")-("<< segments.at(thisSeg).getEndPoint().x << "," << segments.at(thisSeg).getEndPoint().y << ")::(" << segments.at(thatSeg).getStartPoint().x << "," << segments.at(thatSeg).getStartPoint().y << ")-("<< segments.at(thatSeg).getEndPoint().x << "," << segments.at(thatSeg).getEndPoint().y << ")";
                                    qUnprocessed.push(thatSeg);
                                    //take away from pool of raw segs
                                    isSegUsed[thatSeg] = true;
                                    rawSegsLeft--;
                                }
                                else
                                {
                                    //qDebug() << "|" << float(segments.at(thatSeg).getEndPoint().y) <<  "thatSeg End(y)>= intercept" << intercept << "|";
                                    //qDebug() << "|" << int(intercept) << "intercept <= thisSeg End(y)" << segments.at(thisSeg).getEndPoint().y << "|";
                                }
                            }
                        }
                    }
                    else
                    {
                        thatSeg = -1;
                    }
                }

                //add thisSeg to CandidateVector
                candidate_segments.push_back(segments.at(thisSeg));
            }//while (!qUnprocessed->empty())
            //qDebug() << "Candidate ready...";
            //HEURISTICS FOR ADDING THIS CANDIDATE AS A ROBOT CANDIDATE
            if ( max_x - min_x >= 0 &&                                               // width  is non-zero
                 max_y - min_y >= 0 &&                                               // height is non-zero
                 (float)(max_x - min_x) / (float)(max_y - min_y) <= max_aspect &&    // Less    than specified landscape aspect
                 (float)(max_x - min_x) / (float)(max_y - min_y) >= min_aspect &&    // greater than specified portrait aspect
                 segCount >= min_segments                                    // greater than minimum amount of segments to remove noise
                 )
            {
                //qDebug() << "CANDIDATE FINISHED::" << segCount << " segments, aspect:" << ( (float)(max_x - min_x) / (float)(max_y - min_y)) << ", Coords:(" << min_x << "," << min_y << ")-(" << max_x << "," << max_y << "), width: " << (max_x - min_x) << ", height: " << (max_y - min_y);
                int max_col = 0;
                for (int i = 0; i < (int)validColours.size(); i++)
                {
                    if (i != max_col && colourHistogram[i] > colourHistogram[max_col])
                        max_col = i;
                }
                ObjectCandidate temp(min_x, min_y, max_x, max_y, validColours.at(max_col), candidate_segments);
                candidateList.push_back(temp);
            }
	    delete colourHistogram;
        }//while(rawSegsLeft)

    }//if (!segments.empty())
    return candidateList;
}

std::vector<ObjectCandidate> Vision::classifyCandidatesDBSCAN(std::vector< TransitionSegment > segments,
                                        std::vector<Vector2<int> >&fieldBorders,
                                        std::vector<unsigned char> validColours,
                                        int spacing,
                                        float min_aspect, float max_aspect, int min_segments)
{
    std::vector<ObjectCandidate> candidateList;



    return candidateList;
}

bool Vision::isValidColour(unsigned char colour, std::vector<unsigned char> colourList)
{
    bool result = false;
    if (colourList.size())
    {
        std::vector<unsigned char>::const_iterator nextCol = colourList.begin();
        for (;nextCol != colourList.end(); nextCol++)
        {
            if (colour == *(nextCol.base()))
            {
                result = true;
                break;
            }
        }
    }
    return result;
}

int Vision::findYFromX(std::vector<Vector2<int> >&points, int x)
{

    int y = 0;
    int left_x = -1;
    int left_y = 0;
    int right_x = -1;
    int right_y = 0;
    std::vector< Vector2<int> >::const_iterator nextPoint = points.begin();

    for(; nextPoint != points.end(); nextPoint++)
    {
        if (nextPoint->x == x)
        {
            return nextPoint->y;
        }

        if (left_x < nextPoint->x && nextPoint->x < x)
        {
            left_x = nextPoint->x;
            left_y = nextPoint->y;
        }

        if ( (right_x > nextPoint->x || right_x < 0) && nextPoint->x > x)
        {
            right_x = nextPoint->x;
            right_y = nextPoint->y;
        }

    }
    //qDebug() << "findYFromX" << y;
    if (right_x - left_x > 0)
        y = left_y + (right_y-left_y) * (x-left_x) / (right_x-left_x);
    //qDebug() << "findYFromX" << y;
    return y;
}

int Vision::findInterceptFromPerspectiveFrustum(std::vector<Vector2<int> >&points, int current_x, int target_x, int spacing)
{
    int height = currentImage->getHeight();

    int intercept = 0;
    if (current_x == target_x)
    {
        //qDebug() << "Intercept -1 =";
        return -1;
    }

    int y = findYFromX(points, current_x);
    int diff_x = 0;
    int diff_y = height - y;

    if (current_x < target_x)
    {
        diff_x = target_x - current_x;
    }

    if (target_x < current_x)
    {
        diff_x = current_x - target_x;
    }

    if (diff_x > spacing)
    {
        intercept = height;
    }
    else if ( diff_x > spacing/2)
    {
        intercept = y + diff_y/2;
    }
    else if ( diff_x > spacing/4)
    {
        intercept = y + diff_y/4;
    }
    else
    {
        intercept = y;
    }

    //qDebug() << "findInterceptFromPerspectiveFrustum intercept:"<<intercept<<" {y:"<< y << ", height:" << currentImage->height() << ", spacing:" << spacing << ", target_x:" << target_x << ", current_x:" << current_x << "}";
    if (intercept > height)
        intercept = -2;
    return intercept;
}

bool Vision::sortTransitionSegments(TransitionSegment a, TransitionSegment b)
{
    return (a.getStartPoint().x < b.getStartPoint().x || (a.getStartPoint().x == b.getStartPoint().x && a.getEndPoint().y <= b.getStartPoint().y));
}

bool Vision::checkIfBufferSame(boost::circular_buffer<unsigned char> cb)
{

    unsigned char currentClass = cb[0];
    for (unsigned int i = 1; i < cb.size(); i++)
    {
        if(cb[i] != currentClass)
        {
            return false;
        }
    }
    return true;

}

std::vector< ObjectCandidate > Vision::ClassifyCandidatesAboveTheHorizon(   std::vector< TransitionSegment > horizontalsegments,
                                                                            std::vector<unsigned char> validColours,
                                                                            int spacing,
                                                                            int min_segments)
{
    std::vector< ObjectCandidate > candidates;
    std::vector< TransitionSegment > tempSegments;
    bool usedSegments[horizontalsegments.size()];

    for (int i = 0; i < (int)horizontalsegments.size(); i++)
    {
        usedSegments[i] = false;
    }

    int Xstart, Xend, Ystart, Yend;
    //Work Backwards: As post width is acurrate at bottom (no crossbar)
    //ASSUMING EVERYTHING IS ALREADY ORDERED
    for(int i = horizontalsegments.size(); i > 0; i--)
    {
        tempSegments.clear();
        std::vector<int> tempUsedSegments;
        if(!isValidColour(horizontalsegments[i].getColour(), validColours))
        {
            continue;
        }
        //Setting up:
        if(usedSegments[i] != false)
        {
            continue;
        }
        Ystart = horizontalsegments[i].getStartPoint().y;
        Yend = horizontalsegments[i].getEndPoint().y;
        Xstart = horizontalsegments[i].getStartPoint().x;
        Xend = horizontalsegments[i].getEndPoint().x;
        tempSegments.push_back(horizontalsegments[i]);
        tempUsedSegments.push_back(i);
        int nextSegCounter = i+1;

        //We want to stop searching when it leaves the line
        //Searching for a new Xend, close to this current Xstart
        //Then update with new xstart
        //qDebug() << "While:";
        while(horizontalsegments[nextSegCounter].getStartPoint().y ==  Yend && nextSegCounter >0)
        {
            if(usedSegments[nextSegCounter] != false)
            {
                nextSegCounter--;
                continue;
            }
            if(!isValidColour(horizontalsegments[nextSegCounter].getColour(), validColours))
            {
                nextSegCounter--;
                continue;
            }
            if(horizontalsegments[nextSegCounter].getEndPoint().x     < Xstart - spacing/2
               && horizontalsegments[nextSegCounter].getEndPoint().x  > Xstart + spacing/2)
            {
                //Update with new info
                tempSegments.push_back(horizontalsegments[nextSegCounter]);
                tempUsedSegments.push_back(nextSegCounter);
                if (horizontalsegments[nextSegCounter].getStartPoint().x < Xstart)
                {
                    Xstart = horizontalsegments[nextSegCounter].getStartPoint().x;
                }

            }
            nextSegCounter--;
        }
        //Once all the segments on same line has been found scan between
        //(Xstart-spacing) and (Xend+spacing) for segments that are likely to be above the
        //Starting from your where you left off in horzontal count
        //qDebug() << "for:" << nextSegCounter;
        for(int j = nextSegCounter; j > 0; j--)
        {
            if(usedSegments[j] != false)
            {
                continue;
            }
            if(!isValidColour(horizontalsegments[j].getColour(), validColours))
            {
                continue;
            }
            if(horizontalsegments[j].getStartPoint().y < Ystart - spacing*4)
            {
                break;
            }
            if(horizontalsegments[j].getStartPoint().x   > Xstart - spacing/4
               && horizontalsegments[j].getEndPoint().x  < Xend + spacing/4)
            {
                if (horizontalsegments[j].getStartPoint().x < Xstart)
                {
                    Xstart = horizontalsegments[j].getStartPoint().x;
                }
                if (horizontalsegments[j].getEndPoint().x > Xend)
                {
                    Xend = horizontalsegments[j].getEndPoint().x;
                }
                if(horizontalsegments[j].getStartPoint().y < Ystart)
                {
                    Ystart = horizontalsegments[j].getEndPoint().y;
                }
                tempSegments.push_back(horizontalsegments[j]);
                tempUsedSegments.push_back(j);
            }

        }
        //qDebug() << "About: Creating candidate: " << Xstart << ","<< Ystart<< ","<< Xend<< ","<< Yend << " Size: " << tempSegments.size();
        //Create Object Candidate if greater then the minimum number of segments
        if((int)tempSegments.size() >= min_segments && Yend - Ystart > spacing && Xend - Xstart > spacing/4)
        {
            //qDebug() << "Creating candidate: " << Xstart << ","<< Ystart<< ","<< Xend<< ","<< Yend << " Size: " << tempSegments.size();

            ObjectCandidate tempCandidate(Xstart, Ystart, Xend, Yend, validColours[0], tempSegments);
            candidates.push_back(tempCandidate);
            while (!tempUsedSegments.empty())
            {
                usedSegments[tempUsedSegments.back()] = true;
                tempUsedSegments.pop_back();
            }
        }

    }
    //qDebug() << "candidate size: " << candidates.size();
    return candidates;
}

std::vector<LSFittedLine> Vision::DetectLines(ClassifiedSection* scanArea,int spacing)
{
    //qDebug() << "Forming Lines:" << endl;
    LineDetection LineDetector;
    int image_width = currentImage->getWidth();
    int image_height = currentImage->getHeight();
    LineDetector.FormLines(scanArea,image_width,image_height,spacing);
    std::vector<CornerPoint> cornerPoints= LineDetector.cornerPoints;
    std::vector<LSFittedLine> fieldLines= LineDetector.fieldLines;
    //qDebug() << "Detected: " <<  fieldLines.size() << " Lines, " << cornerPoints.size() << " Corners." <<endl;
    return fieldLines;
}

Circle Vision::DetectBall(std::vector<ObjectCandidate> FO_Candidates)
{
    //debug<< "Vision::DetectBall" << endl;

    Ball BallFinding;

    //qDebug() << "Vision::DetectBall : Ball Class created" << endl;
    int width = currentImage->getWidth();
    int height = currentImage->getHeight();
    //qDebug() << "Vision::DetectBall : getting Image sizes" << endl;
    //qDebug() << "Vision::DetectBall : Init Ball" << endl;

    Circle ball;
    ball.isDefined = false;
    if (FO_Candidates.size() <= 0)
    {
        return ball;
    }
    //qDebug() << "Vision::DetectBall : Find Ball" << endl;
    ball = BallFinding.FindBall(FO_Candidates, AllFieldObjects, this, height, width);
    //qDebug() << "Vision::DetectBall : Finnised FO_Ball" << endl;
    if(ball.isDefined)
    {
        //debug<< "Vision::DetectBall : Update FO_Ball" << endl;
        Vector2<int> viewPosition;
        Vector3<float> sphericalError;
        Vector3<float> sphericalPosition;
        viewPosition.x = (int)round(ball.centreX);
        viewPosition.y = (int)round(ball.centreY);
        double ballDistanceFactor=EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS()*ORANGE_BALL_DIAMETER;
        float BALL_OFFSET = 0;
        float distance = (float)(ballDistanceFactor/(2*ball.radius)+BALL_OFFSET);
        float bearing = (float)CalculateBearing(viewPosition.x);
        float elevation = (float)CalculateElevation(viewPosition.y);
        sphericalPosition[0] = distance;
        sphericalPosition[1] = bearing;
        sphericalPosition[2] = elevation;
        //AllFieldObjects->mobileFieldObjects[FieldObjects::FO_BALL].UpdateVisualObject(sphericalPosition,sphericalError,viewPosition);
        //qDebug() << "Setting FieldObject Distance:" << distance;
        //qDebug() << "FO_MOBILE size" << AllFieldObjects->mobileFieldObjects.size();
        //qDebug() << "FO_Stationary size" << AllFieldObjects->stationaryFieldObjects.size();
        AllFieldObjects->mobileFieldObjects[FieldObjects::FO_BALL].UpdateVisualObject(sphericalPosition, sphericalError, viewPosition, currentImage->m_timestamp);
        //ballObject.UpdateVisualObject(sphericalPosition,sphericalError,viewPosition);
        //qDebug() << "Setting FieldObject:" << AllFieldObjects->mobileFieldObjects[FieldObjects::FO_BALL].isObjectVisible();
        /*debug    << "At: Distance: " << AllFieldObjects->mobileFieldObjects[FieldObjects::FO_BALL].measuredDistance()
                    << " Bearing: " << AllFieldObjects->mobileFieldObjects[FieldObjects::FO_BALL].measuredBearing()
                    << " Elevation: " << AllFieldObjects->mobileFieldObjects[FieldObjects::FO_BALL].measuredElevation() << endl;*/

    }

    //qDebug() << "Vision::DetectBall : Finnised" << endl;
    return ball;

}

void Vision::DetectGoals(std::vector<ObjectCandidate>& FO_Candidates,std::vector<ObjectCandidate>& FO_AboveHorizonCandidates,std::vector< TransitionSegment > horizontalSegments)
{
    int width = currentImage->getWidth();
    int height = currentImage->getHeight();
    GoalDetection goalDetector;
    goalDetector.FindGoal(FO_Candidates, FO_AboveHorizonCandidates, AllFieldObjects, horizontalSegments, this,height,width);
    return;
}

double Vision::CalculateBearing(double cx){
    double FOVx = deg2rad(45.0f); //Taken from Old Globals
    return atan( (currentImage->getHeight()/2-cx) / ( (currentImage->getWidth()/2) / (tan(FOVx/2.0)) ) );
}


double Vision::CalculateElevation(double cy){
    double FOVy = deg2rad(34.45f); //Taken from Old Globals
    return atan( (currentImage->getHeight()/2-cy) / ( (currentImage->getHeight()/2) / (tan(FOVy/2.0)) ) );
}

double Vision::EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS()
{
    double FOVx = deg2rad(45.0f); //Taken from Old Globals
    return (0.5*currentImage->getWidth())/(tan(0.5*FOVx));
}
