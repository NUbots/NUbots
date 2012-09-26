#ifndef FIELDLINE_H
#define FIELDLINE_H

#include "visionfieldobject.h"
#include "Tools/Math/Line.h"

class FieldLine : public VisionFieldObject
{
public:
    FieldLine(const Line& line = Line());
    
    //! @brief Stream output for labelling purposes
    void printLabel(ostream& out) {out << getVFOName(FIELDLINE) << " " << m_location_pixels << " " << m_size_on_screen << endl;}
};

#endif // FIELDLINE_H
