/*!
  @file GLDisplay.h
  @brief Declaration of NUbots Vision class.
  @author Steven Nicklin
  @author Aaron Wong
*/

#ifndef GLDISPLAY_H
#define GLDISPLAY_H

#include <QGLWidget>
#include <QMouseEvent>
#include <QColor>

class OpenglManager;

/*!
  @brief Widget used to display images related to images captured by the robots camera,
  as well as subsequent vision processing done on this image.
  */
class GLDisplay : public QGLWidget
{
Q_OBJECT
public:
    /*!
      @brief Constructor.
      @param parent The parent widget.
      @param shareWidget Another 'QGLWidget' with which this one will share textures and
      drawing lists.
      */
    GLDisplay(QWidget *parent = 0,const OpenglManager * shareWidget = 0);
    /*!
      @brief Destructor.
      */
    ~GLDisplay();

    /*!
      @brief Classed used to represent a layer.

      The images displayed in the GLDisplay class are made up of layers. Each window has a primary
      Layer and additional overlay layers. These layers have adjustable alphas when drawn and also
      other parameters to determine when they should and should not be drawn.
      */
    class Layer
    {
    public:
        //! Determines if this particualar layer has been enabled for drawing.
        bool enabled;
        //! The drawing colour of this layer (Includes alpha for transparency).
        QColor colour;
        //! The ID of the display layer type.
        int displayID;
        //! The OpenGL command used to draw this layer.
        GLuint displayCommand;
        //! If True there is a command that can be used to draw this layer. If False there is not.
        bool hasDisplayCommand;
    };

    //! List of the display types
    enum display
    {
        unknown,
        rawImage,
        classifiedImage,
        classificationSelection,
        horizonLine,
        greenHorizonScanPoints,
        greenHorizonPoints,
        horizontalScanPath,
        verticalScanPath,
        numDisplays
    };

    //! Returns the minimum desired size for the window
    QSize minimumSizeHint() const;
    //! Returns the most desired size for the window
    QSize sizeHint() const;

public slots:
    /*!
      @brief Updates the display data.
      @param displayID The id of the display being updated.
      @param newDisplay The new OpenGL instructio that should be used to draw the display.
      @param width The width of this new display.
      @param height The height of this new display.
      */
    void updatedDisplay(int displayID, GLuint newDisplay, int width, int height);
    /*!
      @brief Sets the primary display for the window.
      @param displayID The id of the new primary display.
      */
    void setPrimaryDisplay(int displayID);
    /*!
      @brief Sets the primary display for the window and sets it to draw with a colour other than the default.
      @param displayID The id of the new primary display.
      @param drawingColour The colour to use when drawing this display.
      */
    void setPrimaryDisplay(int displayID, QColor drawingColour);
    /*!
      @brief Used to enable / disable the drawing of an overlay.
      @param displayID The id of the overlay to enable / disable.
      @param enabled If True the drawing of the overlay is enabled. If False it is disabled.
      */
    void setOverlayDrawing(int displayID, bool enabled);
    /*!
      @brief Used to enable / disable the drawing of an overlay and to set an alpha (tansparency) level.
      @param displayID The id of the overlay to enable / disable.
      @param enabled If True the drawing of the overlay is enabled. If False it is disabled.
      @param alpha The new alpha setting to use for this layer.
      */
    void setOverlayDrawing(int displayID, bool enabled, float alpha);
    /*!
      @brief Used to enable / disable the drawing of an overlay and to set a drawing colour.
      @param displayID The id of the overlay to enable / disable.
      @param enabled If True the drawing of the overlay is enabled. If False it is disabled.
      @param drawingColour The new drawing colour to use for this overlay.
      */
    void setOverlayDrawing(int displayID, bool enabled, QColor drawingColour);


signals:
    /*!
      @brief Returns the selected pixel in image coordinates when selected with a left mouse button click.
      @param x The pixels x coordinate
      @param y The pixels y coordinate
      */
    void selectPixel(int x,int y);
    /*!
      @brief Returns the selected pixel in image coordinates when selected with a right mouse button click.
      @param x The pixels x coordinate
      @param y The pixels y coordinate
      */
    void rightSelectPixel(int x,int y);
    /*!
      @brief Returns the selected pixel in image coordinates when selected with a left mouse button click
      with the shift key pressed.
      @param x The pixels x coordinate
      @param y The pixels y coordinate
      */
    void shiftSelectPixel(int x,int y);
    /*!
      @brief Returns the selected pixel in image coordinates when selected with a left mouse button click
      with the ctrl key pressed.
      @param x The pixels x coordinate
      @param y The pixels y coordinate
      */
    void ctrlSelectPixel(int x,int y);

protected:
        void initializeGL();
        void paintGL();
        void resizeGL(int width, int height);
private:
    int imageWidth; //!< The width of the windows current primary display
    int imageHeight; //!< The height of the windows current primary display
    Layer primaryLayer; //!< The current primary layer.
    Layer overlays[numDisplays]; //!< Array of all of the layers.
    //! Overriden function for the mouse press event.
    void mousePressEvent(QMouseEvent * mouseEvent);
    //! Overriden function for the mouse move event.
    void mouseMoveEvent(QMouseEvent * mouseEvent);
    /*!
      @brief Function used to calculate the image coordinates from the screen coordinates.
      @param mouseEvent The mouse event from the selection.
      */
    QPoint calculateSelectedPixel(QMouseEvent * mouseEvent);

    /*!
      @brief Returns the window title for the gien display type.
      @param displayID Id of the display whose title is required.
      @return The title of the display type.
      */
    static const QString getWindowTitle(int displayID)
    {
        switch(displayID)
        {
            case rawImage:
                return QString("Raw Image");
            case classifiedImage:
                return QString("Classified Image");
            case horizonLine:
                return QString("Horizon");
            case classificationSelection:
                return QString("Classification Selection");
            case greenHorizonScanPoints:
                return QString("Green Horizon Scan");
            default:
                return QString("Unknown Image");
        }
    }

    /*!
      @brief gets the default colour for the given display.
      @param displayID Id of the display whose default colour is required.
      @return The default colour for the display.
      */
    static const QColor getDefaultColour(int displayID)
    {
        switch(displayID)
        {
            case rawImage:
                return QColor(255,255,255);
            case classifiedImage:
                return QColor(255,255,255);
            case horizonLine:
                return QColor(255,0,255);
            case classificationSelection:
                return QColor(255,255,255);
            case greenHorizonScanPoints:
                return QColor(0,255,0);
            default:
                return QColor(255,255,255);;
        }
    }
};

#endif // GLDISPLAY_H
