#ifndef LINEANDCIRCLEDETECTOR_H
#define LINEANDCIRCLEDETECTOR_H

#include "Vision/basicvisiontypes.h"
#include "Vision/Modules/circledetector.h"
#include "Vision/Modules/linedetector.h"
#include "Vision/Modules/cornerdetector.h"

using namespace Vision;

class FieldPointDetector
{
public:
    FieldPointDetector(LineDetector* line_detector = NULL, CircleDetector* circle_detector = NULL, CornerDetector* corner_detector = NULL);

    void run() const;

    LineDetector* m_line_detector;
    CircleDetector* m_circle_detector;
    CornerDetector* m_corner_detector;
};

#endif // LINEANDCIRCLEDETECTOR_H
