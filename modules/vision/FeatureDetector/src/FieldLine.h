#ifndef FIELDLINE_H
#define FIELDLINE_H

#include <nuclear>
#include <armadillo>
#include <vector>

#include "VisionFieldObject.h"

class FieldLine : public VisionFieldObject {
public:
    FieldLine(const LSFittedLine& screenLine, const LSFittedLine& groundLine);
    FieldLine(const arma::vec2<NUPoint>& endPoints);

    void set(const LSFittedLine& screenLine, const LSFittedLine& groundLine);
    void set(const arma::vec2<NUPoint>& endPoints);

    Line getScreenLineEquation() const;
    Line getGroundLineEquation() const;
    arma::vec2 getEndPoints() const;
    
    // Dummy until localisation can handle lines.
    bool addToExternalFieldObjects(FieldObjects *fieldobjects, float timestamp) const;

    //! @brief Calculation of error for optimisation
    virtual double findScreenError(VisionFieldObject* other) const;
    virtual double findGroundError(VisionFieldObject* other) const;

    //! @brief output stream operator
    friend std::ostream& operator<< (std::ostream& output, const FieldLine& line);

    //! @brief output stream operator for a std::vector of FieldLines
    friend std::ostream& operator<< (std::ostream& output, const std::vector<FieldLine>& lines);

private:
    Line m_screenLine;
    Line m_groundLine;
    arma::vec2<NUPoint> m_endPoints;
};

#endif // FIELDLINE_H
