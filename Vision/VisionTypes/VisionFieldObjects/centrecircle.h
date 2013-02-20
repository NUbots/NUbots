#ifndef CENTRECIRCLE_H
#define CENTRECIRCLE_H

#include "visionfieldobject.h"
#include "Tools/Math/Circle.h"

class CentreCircle  : public VisionFieldObject
{
public:
    CentreCircle();
    CentreCircle(Point centre, double ground_radius, Vector2<double> screen_size);
    ~CentreCircle();

    virtual bool addToExternalFieldObjects(FieldObjects* fieldobjects, float timestamp) const;

    //! @brief Stream output for labelling purposes
    virtual void printLabel(ostream& out) const;
    //! @brief Brief stream output for labelling purposes
    virtual Vector2<double> getShortLabel() const;

    //! @brief Calculation of error for optimisation
    virtual double findError(const Vector2<double>& measured) const;

    double getGroundRadius() const {return m_ground_radius;}

private:
    double m_ground_radius;
};

#endif // CENTRECIRCLE_H
