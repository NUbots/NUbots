#ifndef OPENGLMANAGER_H
#define OPENGLMANAGER_H

#include <QObject>
#include <QGLWidget>
#include "gldisplay.h"
#include "Tools/Math/Vector2.h"

class NUimage;
class ClassifiedImage;
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
        /*!
          @brief Accepts a new raw robot image and maps it to display instructions.
          @param newImage The new raw image.
          */
        void newRawImage(NUimage* newImage);
        /*!
          @brief Accepts a new classified robot image and maps it to display instructions.
          @param newImage The new classified image.
          */
        void newClassifiedImage(ClassifiedImage* newImage);
        /*!
          @brief Accepts a new horizon line object and maps it to display instructions.
          @param newHorizon The new horizon line.
          */
        void newHorizon(Horizon* newHorizon);
        /*!
          @brief Accepts a new classified colour selection image and maps it to display instructions.
          @param newImage The new classified selection image.
          */
        void newClassificationSelection(ClassifiedImage* newImage);
        /*!
          @brief Accepts new point objects and maps it to display instructions.
          @param newpoints The new points.
          */
        void newGreenpoints(std::vector< Vector2<int> > newpoints);

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
        void createDrawTextureImage(QImage& image, int displayId);
        void drawHollowCircle(float cx, float cy, float r, int num_segments);
        void drawSolidCircle(float cx, float cy, float r, int num_segments);

};

#endif // OPENGLMANAGER_H
