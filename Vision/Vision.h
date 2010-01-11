/*!
  @file Vision.h
  @brief Declaration of NUbots Vision class.
*/

#ifndef VISION_H
#define VISION_H

#include <vector>
#include <boost/circular_buffer.hpp>
#include "Tools/Math/Vector2.h"
#include "Kinematics/Horizon.h"
#include "Tools/Image/ClassifiedImage.h"
#include "ClassifiedSection.h"
#include "ScanLine.h"
#include "TransitionSegment.h"

class NUimage;

//! Contains vision processing tools and functions.
class Vision
{
    public:

    const static unsigned int VERT_JOIN_LIMIT = 3;
    const static unsigned int HORZ_JOIN_LIMIT = 2;

    //! Default constructor.
    Vision();
    //! Destructor.
    ~Vision();
    /*!
      @brief Produce a classified.

      Primarily used for display when debugging and constructing a lookup table.
      @param targetImage The target classification image that will be written to.
      @param sourceImage The raw image to be classified.
      @param lookUpTable The colour classification lookup table. This table maps colours
      from the raw source image into the classified colour space.
      */
    void classifyImage(ClassifiedImage &targetImage, const NUimage* sourceImage, const unsigned char *lookUpTable);
    /*!
      @brief Classifies an individual pixel.
      @param x The x coordinate of the pixel to be classified.
      @param y The y coordinate of the pixel to be classified.
      @return Returns the classfied colour index for the given pixel.
      */
    inline unsigned char classifyPixel(int x, int y);

    /*!
      @brief Joins segments to create a joined segment clusters that represent candidate robots
      @param segList The segList is a vector of TransitionSegments after field lines have been rejected
    */
    void classifyRobotCandidates(std::vector< TransitionSegment > segments);

    /*!
      @brief Returns true when the colour passed in is a valid robot colour
      @param colour The colour value that needs to be checked if it is a robot colour
      @return bool True when the colour passed in is an assigned robot colour
    */
    bool Vision::isRobotColour(unsigned char colour);

    std::vector<Vector2<int> > findGreenBorderPoints(const NUimage* sourceImage, const unsigned char *lookUpTable, int scanSpacing, Horizon* horizonLine);
    std::vector<Vector2<int> > getConvexFieldBorders(std::vector<Vector2<int> >& fieldBorders);
    std::vector<Vector2<int> > interpolateBorders(std::vector<Vector2<int> >& fieldBorders, int scanSpacing);

    ClassifiedSection* horizontalScan(std::vector<Vector2<int> >&fieldBoarders, int scanSpacing);
    ClassifiedSection* verticalScan(std::vector<Vector2<int> >&fieldBoarders, int scanSpacing);
    void ClassifyScanArea(ClassifiedSection* scanArea);


    private:    
    const NUimage* currentImage; //!< Storage of a pointer to the raw colour image.
    const unsigned char* currentLookupTable; //!< Storage of a pointer to the current colour lookup table.

    bool checkIfBufferSame(boost::circular_buffer<unsigned char> cb);
};
#endif // VISION_H
