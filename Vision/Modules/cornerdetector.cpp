#include "cornerdetector.h"
#include "visionblackboard.h"
#include "Vision/VisionTypes/VisionFieldObjects/fieldline.h"

CornerDetector::CornerDetector()
{
}

vector<CornerPoint> CornerDetector::run() const
{
    const vector<FieldLine>& lines = VisionBlackboard::getInstance()->getLines();
    vector<FieldLine>::const_iterator it1, it2;

    for(it1 = lines.begin(); it1 < lines.end()-1; it1++) {
        for(it2 = it1+1; it2 < lines.end(); it2++) {

        }
    }
}
