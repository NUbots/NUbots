#ifndef GOAL_H
#define GOAL_H

#include <nuclear>
#include <armadillo>

#include "VisionFieldObject.h"
#include "Quad.h"

class Goal : public VisionFieldObject {
public:
    Goal(VFO_ID id = INVALID, const Quad& corners = Quad(), bool known = false);

    void setBase(arma::vec2 base);

    //! @brief reutns the pixel locations of the corners.
    const Quad& getQuad() const;

    /*!
      @brief pushes the goal to the external field objects.
      @param fieldobjects a pointer to the global list of field objects.
      @param timestamp the image timestamp.
      @return the success of the operation.
      */
    bool addToExternalFieldObjects(FieldObjects *fieldobjects, float timestamp) const;

    //! @brief applies a series of checks to decide if the goal is valid.
    bool check() const;
        
    //! @brief Stream output for labelling purposes
    void printLabel(std::ostream& out) const {
      out << VFOName(m_id) << " " << m_location << " " << m_size_on_screen;
    }

    virtual double findScreenError(VisionFieldObject* other) const;
    virtual double findGroundError(VisionFieldObject* other) const;

    //! @brief output stream operator.
    friend std::ostream& operator<< (std::ostream& output, const Goal& g);

    //! @brief output stream operator for a vector of goals.
    friend std::ostream& operator<< (std::ostream& output, const std::vector<Goal>& g);    
    
private:
    /*!
      @brief calculates various positions values of the goal.
      @return whether the goal is valid.
      */
    bool calculatePositions();

    /*!
      @brief calculates distance to the goal based on the global goal distance metric.
      @param bearing the angle between the goal and the image centre in the xy plane.
      @param elevation the angle between the goal and the image centre in the xz plane.
      @return the distance to the goal in cm.
      */
//    double distanceToGoal(double bearing, double elevation);

public:
    bool m_known;

private:
    Quad m_corners;                                       //! @variable pixel locations of the corners
    NUPoint m_d2pLocation, m_widthLocation, m_heightLocation;
    double m_widthDistance, m_heightDistance;
    bool m_offTop, m_offBottom, m_offSide;

//public:
//    double width_dist,
//           d2p;
};

#endif // GOAL_H
