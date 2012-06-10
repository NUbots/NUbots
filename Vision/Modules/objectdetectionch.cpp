/**
*   @name   ObjectDetectionCH
*   @file   objectdetectionch.cpp
*   @brief  basic object detection by checking breaks in green horizon.
*   @author David Budden
*   @date   22/02/2012
*/

#include "objectdetectionch.h"

#include "Vision/visionconstants.h"

void ObjectDetectionCH::detectObjects()
{
    #if VISION_HORIZON_VERBOSITY > 1
        debug << "ObjectDetectionCH::detectObjects() - Begin" << endl;
    #endif
    // get blackboard instance
    VisionBlackboard* vbb = VisionBlackboard::getInstance();
    const NUImage& img = vbb->getOriginalImage();
    unsigned int height = img.getHeight();

    const vector<PointType>& horizon_points = vbb->getGreenHorizon().getInterpolatedSubset(VisionConstants::VERTICAL_SCANLINE_SPACING);

    vector<PointType> object_points;

    object_points.reserve(horizon_points.size());

    cv::Mat mean, std_dev;
    meanStdDev(cv::Mat(horizon_points), mean, std_dev);

    //cout << mean << " " << std_dev << endl;

    // for each point in interpolated list
    for (unsigned int x = 0; x < horizon_points.size(); x++) {
        unsigned int green_top = 0;
        unsigned int green_count = 0;

        // if bottom of image, assume object
        if (static_cast<unsigned int>(horizon_points.at(x).y) == height-1) {
            object_points.push_back(PointType(horizon_points.at(x).x, height-1));
        }
        else {
            // scan from point to bottom of image
            for (unsigned int y = horizon_points.at(x).y; y < height; y++) {
                if (isPixelGreen(img, horizon_points.at(x).x, y)){
                    if (green_count == 1) {
                        green_top = y;
                    }
                    green_count++;
                    // if VER_THRESHOLD green pixels found outside of acceptable range, add point
                    if (green_count == VER_THRESHOLD) {
                        if (green_top > mean.at<double>(1) + OBJECT_THRESHOLD_MULT*std_dev.at<double>(1) + 1) {
                            //cout << "OBJECT: (" << horizon_points->at(x).x << ", " << y << ")" << endl;
                            object_points.push_back(PointType(horizon_points.at(x).x, y));
                        }
                        break;
                    }
                }
                // not green - reset
                else {
                    green_count = 0;
                }

                // if bottom reached without green, add bottom point
                if (y == height - 1) {
                    object_points.push_back(PointType(horizon_points.at(x).x, height-1));
                }
            }
        }
    }

    // update blackboard with object points
    vbb->setObjectPoints(object_points);
}


bool ObjectDetectionCH::isPixelGreen(const NUImage& img, int x, int y)
{
    const LookUpTable& LUT = VisionBlackboard::getInstance()->getLUT();
    return ClassIndex::getColourFromIndex(LUT.classifyPixel(img(x,y))) == ClassIndex::green;
}
