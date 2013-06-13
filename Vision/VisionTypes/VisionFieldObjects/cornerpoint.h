#ifndef CORNERPOINT_H
#define CORNERPOINT_H

#include "visionfieldobject.h"

class CornerPoint : public VisionFieldObject
{
public:
    enum TYPE {
        L,
        T,
        X,
        INVALID
    };

public:
    CornerPoint(TYPE type, NUPoint location);

    virtual bool addToExternalFieldObjects(FieldObjects* fieldobjects, float timestamp) const;

    //! @brief Stream output for labelling purposes
    virtual void printLabel(ostream& out) const;

    //! @brief Calculation of error for optimisation
    virtual double findScreenError(VisionFieldObject* other) const;
    virtual double findGroundError(VisionFieldObject* other) const;

    //! @brief output stream operator.
    friend ostream& operator<< (ostream& output, const CornerPoint& c);
    //! @brief output stream operator for a vector of corner points.
    friend ostream& operator<< (ostream& output, const vector<CornerPoint>& c);

private:
    TYPE m_type;
};

#endif // CORNERPOINT_H
