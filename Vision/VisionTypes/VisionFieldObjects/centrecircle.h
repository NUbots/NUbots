#ifndef CENTRECIRCLE_H
#define CENTRECIRCLE_H

#include "visionfieldobject.h"
#include "Tools/Math/Circle.h"

class CentreCircle  : public VisionFieldObject
{
public:
    CentreCircle(Circle ground_equation);

    virtual bool addToExternalFieldObjects(FieldObjects* fieldobjects, float timestamp) const;

    //! @brief Stream output for labelling purposes
    virtual void printLabel(ostream& out) const;
    //! @brief Brief stream output for labelling purposes
    virtual Vector2<double> getShortLabel() const;

    //! @brief Calculation of error for optimisation
    virtual double findError(const Vector2<double>& measured) const;

private:
    Circle m_ground_circle;
};

#endif // CENTRECIRCLE_H
