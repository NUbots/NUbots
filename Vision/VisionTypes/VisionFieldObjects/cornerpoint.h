#ifndef CORNERPOINT_H
#define CORNERPOINT_H

#include "visionfieldobject.h"

class CornerPoint : public VisionFieldObject
{
public:
    enum TYPE {
        L,
        T,
        CROSS,
        INVALID
    };

public:
    CornerPoint(TYPE type, Point location);

    virtual bool addToExternalFieldObjects(FieldObjects* fieldobjects, float timestamp) const;

    //! @brief Stream output for labelling purposes
    virtual void printLabel(ostream& out) const;
    //! @brief Brief stream output for labelling purposes
    virtual Vector2<double> getShortLabel() const;

    //! @brief Calculation of error for optimisation
    virtual double findError(const Vector2<double>& measured) const;

private:
    TYPE m_type;
};

#endif // CORNERPOINT_H
