//#ifndef BEACON_H
//#define BEACON_H

//#include "Vision/VisionTypes/VisionFieldObjects/visionfieldobject.h"
//#include "Vision/VisionTypes/quad.h"

//class Beacon : public VisionFieldObject
//{
//public:
//    Beacon(VFO_ID id = INVALID, const Quad& corners = Quad(0,0,0,0));
    
//    //! @brief reutns the pixel locations of the corners.
//    const Quad& getQuad() const;

//    /*!
//      @brief pushes the beacon to the external field objects.
//      @param fieldobjects a pointer to the global list of field objects.
//      @param timestamp the image timestamp.
//      @return the success of the operation.
//      */
//    bool addToExternalFieldObjects(FieldObjects *fieldobjects, float timestamp) const;
//    //! @brief applies a series of checks to decide if the beacon is valid.
//    bool check() const;

//    //! @brief sets the beacon to be an unkown beacon
//    void setUnknown();
    
//    //! @brief Stream output for labelling purposes
//    void printLabel(ostream& out) const {out << VFOName(m_id) << " " << m_location_pixels << " " << m_size_on_screen;}
//    //! @brief Brief stream output for labelling purposes
//    //void printLabelBrief(ostream& out) const {out << VFOName(m_id) << " " << m_location_pixels;}

//    double findError(const Vector2<double>& measured) const {return sqrt( pow(m_location_pixels.x - measured.x,2) + pow(m_location_pixels.y - measured.y,2));}

//    void render(cv::Mat& mat) const;

//    //! @brief output stream operator
//    friend ostream& operator<< (ostream& output, const Beacon& b);
//    //! @brief output stream operator for a vector of beacons
//    friend ostream& operator<< (ostream& output, const vector<Beacon>& b);
    
//private:
//    /*!
//      @brief calculates various positions values of the beacon.
//      @return whether the beacon is valid.
//      */
//    bool calculatePositions();
//    /*!
//      @brief calculates distance to the beacon based on the global beacon distance metric.
//      @param bearing the angle between the beacon and the image centre in the xy plane.
//      @param elevation the angle between the beacon and the image centre in the xz plane.
//      @return the distance to the beacon in cm.
//      */
//    float distanceToBeacon(float bearing, float elevation);
    
//private:
//    Quad m_corners;                 //! @variable pixel locations of the corners
    
//    float d2p;          //! @variable the distance of the beacon in cm as found by the distance to point method
//    float width_dist;   //! @variable the distance of the beacon in cm as found by the width method.
//};

//#endif // BEACON_H
