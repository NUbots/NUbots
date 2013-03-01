/*! @file openglmanager.h
    @brief Decleration of OpenglManager class.

    @class OpenglManager
    @brief Class used to generate openGL drawing lists to display various vision data.

    OpenglManager is used to convert various visualisations of data
    produced by the NUbots vision system into OpenGL display lists.
    Each display list is converted into a display 'Layer'. These layers
    can be displayed within the GLDisplay widget in any number of
    combinations. By centralising the conversion of data into drawing
    instruction and buffering these instructions within this widget,
    the often lengthy proces of conversion between large sets of data
    and images into drawing instructions is kept to a minimum. The openGL
    instructions produced run entirely on the video hardware when available.

    @author Steven Nicklin

  Copyright (c) 2010 Steven Nicklin

    This file is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This file is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with NUbot.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef OPENGLMANAGER_H
#define OPENGLMANAGER_H

#include <QObject>
#include <QGLWidget>
#include "GLDisplay.h"
#include "Tools/Math/Vector2.h"
#include "VisionOld//TransitionSegment.h"
#include "Tools/Math/LSFittedLine.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "localisationwidget.h"
#include "VisionOld//ObjectCandidate.h"
#include "VisionOld//CornerPoint.h"

class NUImage;
class ClassifiedImage;
class Line;
class LinePoint;
class Horizon;
class NUSensorsData;
class KF;

class OpenglManager : public QGLWidget
{
Q_OBJECT
public:
    /*!
      @brief Standard constructor.
      */
    OpenglManager();
    /*!
      @brief Destructor.
      */
    ~OpenglManager();

    signals:
        /*!
          @brief Updates a display with new drawing instructions.
          @param displayID The id of the display being updated.
          @param newDisplay The new drawing instructions for that display.
          @param width The width of the display.
          @param height The height of the display.
          */
        void updatedDisplay(int displayID, GLuint newDisplay, int width, int height);

    public slots:
        /*!
          @brief Accepts a new raw image and maps it to the default display layer.
          @param newImage The new raw image.
          */
        void setRawImage(const NUImage* newImage){writeNUImageToDisplay(newImage, GLDisplay::rawImage);};
        /*!
          @brief Accepts a new raw image and maps it to display instructions.
          @param newImage The new raw image.
          @param displayId The id of the display layer to write to.
          */
        void writeNUImageToDisplay(const NUImage* newImage, GLDisplay::display displayId);
        /*!
          @brief Accepts a new classified image and maps it to display instructions.
          @param newImage The new classified image.
          @param displayId The id of the display layer to write to.
          */
        void writeClassImageToDisplay(ClassifiedImage* newImage, GLDisplay::display displayId);
        /*!
          @brief Accepts a new line object and maps it to display instructions.
          @param newHorizon The new horizon line.
          @param displayId The id of the display layer to write to.
          */
        void writeLineToDisplay(Line* newLine, GLDisplay::display displayId);
        /*!
          @brief Accepts new point vector and maps it to display instructions.
          @param newpoints The new points.
          @param displayId The id of the display layer to write to.
          */
        void writePointsToDisplay(std::vector< Vector2<int> > newpoints, GLDisplay::display displayId);

        /*!
          @brief Accepts new transition segment vector and maps it to display instructions.
          @param newsegments The new tansition segments.
          @param displayId The id of the display layer to write to.
          */
        void writeTransitionSegmentsToDisplay(std::vector< TransitionSegment > newsegments, GLDisplay::display displayId);

        /*!
          @brief Accepts new object candidate vector and maps it to display.
          @param candidates The new Object Candidates to display.
          @param displayId The id of the display layer to write to.
          */
        void writeCandidatesToDisplay(std::vector< ObjectCandidate > candidates, GLDisplay::display displayId);

        /*!
          @brief Accepts array of new world model lines and maps them to display instructions.
          @param newWMLine The array of new world model lines.
          @param numLines The size of the array of world model lines
          @param displayId The id of the display layer to write to.
          */
        void writeWMLineToDisplay(WMLine* newWMLine, int numLines, GLDisplay::display displayId);

        /*!
          @brief Accepts centre and radius of new world model ball and maps it to display instructions.
          @param x The x ordinate of the centre of the ball.
          @param y The y ordinate of the centre of the ball.
          @param radius The radius of the ball.
          @param displayId The id of the display layer to write to.
          */
        void writeWMBallToDisplay(float x, float y, float radius,GLDisplay::display displayId);

        /*!
          @brief Accepts a display and clears it's previously stored lists.
          @param displayId The display to be cleared.
          */

        void writeCalGridToDisplay(GLDisplay::display displayId);

        /*!
          @brief Accepts a display and clears it's previously stored lists.
          @param displayId The display to be cleared.
          */


        void clearDisplay(GLDisplay::display displayId);

        /*!
          @brief Accepts new LSFittedLine vector and maps it to display instructions.
          @param fieldLines The new LSFittedLines to display.
          @param displayId The id of the display layer to write to.
          */
        void writeFieldLinesToDisplay(std::vector< LSFittedLine > fieldLines, GLDisplay::display displayId);

        /*!
          @brief Accepts new linepoint vector and maps it to display instructions.
          @param linepoints The new LinePoints to display.
          @param displayId The id of the display layer to write to.
          */
        void writeLinesPointsToDisplay(std::vector< LinePoint > linepoints, GLDisplay::display displayId);

        /*!
          @brief Accepts new Corner vector and maps it to display instructions.
          @param corners The new Corner to display.
          @param displayId The id of the display layer to write to.
        */
        void writeCornersToDisplay(std::vector< CornerPoint > Corners, GLDisplay::display displayId);
        /*!
          @brief Accepts Field Objects and maps it to display instructions.
          @param FieldObjects The new fieldObjects to display.
          @param displayId The id of the display layer to write to.
          */
        void writeFieldObjectsToDisplay(FieldObjects* AllObjects, GLDisplay::display displayId);

        /*!
          @brief Test stub for displaying alpha transparent images
          @param displayId The id of the display layer to write to.
          @param image     The image to update the layer with
          */
        void stub(QImage image, GLDisplay::display displayId);
        /*!
        @brief Removes all of the previous display instructions so old display data is not reused.
        */
        void clearAllDisplays();

    public:
        /*!
            @brief Determine if a layer has a valid display instruction.
            @param id The id of the display layer.
            @return True if a valid instruction is available. False if no valid instruction exists.
        */
        bool hasDisplayCommand(int id) const {return displayStored[id];};
        /*!
            @brief Retrieve the display instruction for a display layer.
            @param id The id of the display layer.
            @return The openGL drawing instruction for the specified layer.
        */
        GLuint getDisplayCommand(int id) const {return displays[id];};
        /*!
            @brief Retrieve width of the current displays produced
            @return The width of the images produced in pixels.
        */
        int getWidth() const {return width;};
        /*!
            @brief Retrieve height of the current displays produced
            @return The height of the images produced in pixels.
        */
        int getHeight() const {return height;};
    private:
        int width;                                  //!< Width of the current image.
        int height;                                 //!< Height of the current image.
        GLuint displays[GLDisplay::numDisplays];    //!< Storage for drawing instructions.
        GLuint textures[GLDisplay::numDisplays];    //!< Storage for textures.
        bool displayStored[GLDisplay::numDisplays]; //!< Tracking of drawing instruction storage.
        bool textureStored[GLDisplay::numDisplays]; //!< Tracking of texture storage.
        /*!
          @brief Maps an image to a texture and loads it to the graphics card for display.
          @param image The image to be mapped.
          @param displayId The id of the display layer to which to draw the texture.
          */
        void createDrawTextureImage(const QImage& image, int displayId);
        /*!
          @brief Draw the circumference of a circle.
          @param cx The x coordinate of the circles centre.
          @param cy The y coordinate of the circles centre.
          @param r The radius of the circle.
          @param num_segments The Number of segments to use when constructing the circle.
          */
        void drawHollowCircle(float cx, float cy, float r, int num_segments);
        /*!
          @brief Draw a solid circle.
          @param cx The x coordinate of the circles centre.
          @param cy The y coordinate of the circles centre.
          @param r The radius of the circle.
          @param num_segments The Number of segments to use when constructing the circle.
          */
        void drawSolidCircle(float cx, float cy, float r, int num_segments);

        void  drawEllipse(float cx, float cy, float xradius, float yradius);

};

#endif // OPENGLMANAGER_H
