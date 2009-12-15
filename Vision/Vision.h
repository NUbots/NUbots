/*!
  @file Vision.h
  @brief Declaration of NUbots Vision class.
*/

#ifndef VISION_H
#define VISION_H

#include <vector>
#include "Tools/Math/Vector2.h"
#include "Kinematics/Horizon.h"
#include "Tools/Image/ClassifiedImage.h"

class NUimage;

//! Contains vision processing tools and functions.
class Vision
{
    public:
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
      @brief Identifies number of robots in Field of View (FoV)
      @param fieldBorders list of (x,y) cartesian points representing points along the field border
      @return Returns the integer number of robots identified and negative when an error occurred
      */
    int countRobots(std::vector<Vector2<int> > &fieldBorders);

    std::vector<Vector2<int> > findGreenBorderPoints(const NUimage* sourceImage, const unsigned char *lookUpTable, int scanSpacing, Horizon* horizonLine);
    std::vector<Vector2<int> > getConvexFieldBorders(std::vector<Vector2<int> >& fieldBorders);
    std::vector<Vector2<int> > interpolateBorders(std::vector<Vector2<int> >& fieldBorders, int scanSpacing);

    std::vector<Vector2<int> > horizontalScan(std::vector<Vector2<int> >&fieldBoarders, int scanSpacing);
    std::vector<Vector2<int> > verticalScan(std::vector<Vector2<int> >&fieldBoarders, int scanSpacing);

    private:    
    const NUimage* currentImage; //!< Storage of a pointer to the raw colour image.
    const unsigned char* currentLookupTable; //!< Storage of a pointer to the current colour lookup table.

};
#endif // VISION_H
