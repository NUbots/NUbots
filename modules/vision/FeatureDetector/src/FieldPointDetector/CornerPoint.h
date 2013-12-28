#ifndef CORNERPOINT_H
#define CORNERPOINT_H

#include <nuclear>
#include <armadillo>
#include <vector>

#include "VisionFieldObject.h"

class CornerPoint : public VisionFieldObject {
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

    //! @brief Calculation of error for optimisation
    virtual double findScreenError(VisionFieldObject* other) const;
    virtual double findGroundError(VisionFieldObject* other) const;

    //! @brief output stream operator.
    friend std::ostream& operator<< (std::ostream& output, const CornerPoint& c);

    //! @brief output stream operator for a vector of corner points.
    friend std::ostream& operator<< (std::ostream& output, const std::vector<CornerPoint>& c);

private:
    TYPE m_type;
};

#endif // CORNERPOINT_H
