#ifndef CENTRECIRCLE_H
#define CENTRECIRCLE_H

#include <nuclear>
#include <armadillo>
#include <vector>

#include "VisionFieldObject.h"

class CentreCircle  : public VisionFieldObject {
public:
    CentreCircle();
    CentreCircle(NUPoint centre, double groundRadius, arma::vec2 screenSize);
    ~CentreCircle();

    virtual bool addToExternalFieldObjects(FieldObjects* fieldobjects, float timestamp) const;

    //! @brief Calculation of error for optimisation
    virtual double findScreenError(VisionFieldObject* other) const;
    virtual double findGroundError(VisionFieldObject* other) const;

    double getGroundRadius() const;

    //! @brief output stream operator.
    friend std::ostream& operator<< (std::ostream& output, const CentreCircle& c);
    //! @brief output stream operator for a vector of CentreCircles.
    friend std::ostream& operator<< (std::ostream& output, const std::vector<CentreCircle>& c);

private:
    double m_groundRadius;
};

#endif // CENTRECIRCLE_H
