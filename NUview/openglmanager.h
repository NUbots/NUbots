#ifndef OPENGLMANAGER_H
#define OPENGLMANAGER_H

#include <QObject>
#include <QGLWidget>
#include "GLDisplay.h"
#include "Tools/Math/Vector2.h"
#include "Vision/TransitionSegment.h"
#include "Tools/Math/LSFittedLine.h"
#include "localisationwidget.h"
#include "Vision/ObjectCandidate.h"

class NUimage;
class ClassifiedImage;
class Line;
class Horizon;

class OpenglManager : public QGLWidget
{
Q_OBJECT
public:

    OpenglManager();
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
        void newRawImage(const NUimage* newImage){writeNUimageToDisplay(newImage, GLDisplay::rawImage);};
        /*!
          @brief Accepts a new raw image and maps it to display instructions.
          @param newImage The new raw image.
          @param displayId The id of the display layer to write to.
          */
        void writeNUimageToDisplay(const NUimage* newImage, GLDisplay::display displayId);
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
        void clearDisplay(GLDisplay::display displayId);

        /*!
          @brief Accepts new LSFittedLine vector and maps it to display instructions.
          @param fieldLines The new LSFittedLines to display.
          @param displayId The id of the display layer to write to.
          */
        void writeFieldLinesToDisplay(std::vector< LSFittedLine > fieldLines, GLDisplay::display displayId);

        /*!
        @brief Removes all of the previous display instructions so old display data is not reused.
        */
        void clearAllDisplays();

    public:
        bool hasDisplayCommand(int id) const {return displayStored[id];};
        GLuint getDisplayCommand(int id) const {return displays[id];};
        int getWidth() const {return width;};
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
          @param displayId The .display Id to associatie this texture with.
          */
        void createDrawTextureImage(const QImage& image, int displayId);
        void drawHollowCircle(float cx, float cy, float r, int num_segments);
        void drawSolidCircle(float cx, float cy, float r, int num_segments);

};

#endif // OPENGLMANAGER_H
