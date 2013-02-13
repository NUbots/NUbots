/*! @file GLDisplay.h
    @brief Declaration of GLDisplay class.

    @class GLDisplay
    @brief GLDisplay is used to combine and display openGL display layers
    produced by an OpenglManager object.

    The GLDisplay class combines display layers with an adjustable
    colour and transparency to create customised views of the data produced
    primarily by the NUbots vision system. This includes data such as images,
    feature points, lines and more.

    @author Steven Nicklin
    @author Aaron Wong

    Copyright (c) 2010 Steven Nicklin, Aaron Wong

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

#ifndef GLDISPLAY_H
#define GLDISPLAY_H

#include <QGLWidget>
#include <QMouseEvent>
#include <QColor>

class OpenglManager;

class GLDisplay : public QGLWidget
{
Q_OBJECT
public:
    static int imageCount; //!< The number of images that have been saved before

    /*!
      @brief Constructor.
      @param parent The parent widget.
      @param shareWidget An OpenglManager producing the display lists used to create displays.
      */
    GLDisplay(QWidget *parent = 0,const OpenglManager * shareWidget = 0);
    /*!
      @brief Destructor.
      */
    ~GLDisplay();

    /*!
      @brief Retrieve the size of the images displayed by the widget.
      @return The size of the images that are displayed.
      */
    QSize imageSize()
    {
        return QSize(imageWidth, imageHeight);
    }
    /*!
      @brief Class used to represent a layer.

      The images displayed in the GLDisplay class are made up of layers. Each window has a primary
      Layer and additional overlay layers. These layers have adjustable alphas when drawn and also
      other parameters to determine when they should and should not be drawn.
      */
    class Layer
    {
    public:
        //! Determines if this layer is the primary layer for the display.
        bool primary;
        //! Determines if this particular layer has been enabled for drawing.
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

    //! List of the display layers
    enum display
    {
        unknown                 = 0,
        rawImage                = 1,
        classifiedImage         = 2,
        classificationSelection = 3,
        horizonLine             = 4,
        greenHorizonScanPoints  = 5,
        greenHorizonPoints      = 6,
        horizontalScanPath      = 7,
        verticalScanPath        = 8,
        Segments                = 9,
        FilteredSegments        = 10,
        Transitions             = 11,
        FieldLines              = 12,
        ObjectCandidates        = 13,
        FieldObjects            = 14,
        EdgeFilter              = 15,
        FFT                     = 16,
        wmRightLeg              = 17,
        wmLeftLeg               = 18,
        wmBall                  = 19,
        CalGrid                 = 20,
        GoalEdgeLinesStart      = 21,
        GoalEdgeLinesEnd        = 22,
        Plot                    = 23,
        numDisplays             = 24
    };

    //! Returns the minimum desired size for the window
    QSize minimumSizeHint() const;
    //! Returns the most desired size for the window
    QSize sizeHint() const;

    /*!
      @brief Restores the windows display settings to those described in the QByteArray
      @param state The state of the display settings to which the window will be set.
      */
    void restoreState(const QByteArray & state);

    /*!
      @brief Returns the current state of the windows display settings.
      @return The QByteArray storing the current display settings.
      */
    QByteArray saveState() const;

    /*!
      @brief Returns the window title for the gien display type.
      @param displayID Id of the display whose title is required.
      @return The title of the display type.
      */
    static const QString getLayerName(int displayID)
    {
        switch(displayID)
        {
        case rawImage:
            return QString("Raw");
        case classifiedImage:
            return QString("Classified");
        case horizonLine:
            return QString("Horizon");
        case classificationSelection:
            return QString("Classification Selection");
        case greenHorizonScanPoints:
            return QString("Green Horizon Scan");
        case greenHorizonPoints:
            return QString("Green Horizon");
        case horizontalScanPath:
            return QString("Horizontal Scan Path");
        case verticalScanPath:
            return QString("Vertical Scan Path");
        case Segments:
            return QString("Unfiltered Segments");
        case FilteredSegments:
            return QString("Filtered Segments");
        case Transitions:
            return QString("Transitions");
        case FieldLines:
            return QString("Field Lines");
        case ObjectCandidates:
            return QString("Field Object Candidates");
        case FieldObjects:
            return QString("Field Objects");
        case EdgeFilter:
            return QString("Edge Filter");
        case FFT:
            return QString("FFT");
        case wmLeftLeg:
            return QString("World Model: Left Leg");
        case wmRightLeg:
            return QString("World Model: Right Leg");
        case wmBall:
            return QString("World Model: Ball");
        case CalGrid:
            return QString("Calibration Grid");
        case GoalEdgeLinesStart:
            return QString("Goal Edge Lines: Start");
        case GoalEdgeLinesEnd:
            return QString("Goal Edge Lines: End");
        case Plot:
            return QString("Plot");
        default:
            return QString("Unknown");
        }
    }

    /*!
      @brief Get the settings for a layer.
      @param layerId The ID of the layer.
      @return The settings for this layer.
      */
    const Layer* getLayerSettings(int layerId)
    {
        return &overlays[layerId];
    }

    /*!
      @brief Get the ID of the primary layer.
      @return The ID of the primary layer. display::unknown if none is set.
      */
    int getPrimaryLayerId()
    {
        if(primaryLayer)
            return primaryLayer->displayID;
        else
            return unknown;
    }

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

    /*!
      @brief Copy the current image displayed to the system clipboard.
      */
    void snapshotToClipboard();

    /*!
      @brief Copy the current image displayed to the executable directory as a png.
      */
    void snapshotToFile();

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
    //! Initialise the openGL display area.
    void initializeGL();
    //! Draw the openGL display area.
    void paintGL();
    //! Resize the openGL display area.
    void resizeGL(int width, int height);
private:
    int imageWidth; //!< The width of the windows current primary display
    int imageHeight; //!< The height of the windows current primary display

    Layer* primaryLayer; //!< The current primary layer.
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
            return QColor(255,0,0);
        case greenHorizonPoints:
            return QColor(0,255,127);
        case horizontalScanPath:
            return QColor(255,0,0);
        case verticalScanPath:
            return QColor(0,255,127);
        case Segments:
            return QColor(255,255,255);
        case FilteredSegments:
            return QColor(255,255,255);
        case Transitions:
            return QColor(255,255,255);
        case ObjectCandidates:
            return QColor(255,128,64);
        case EdgeFilter:
            return QColor(255,255,255);
        case FFT:
            return QColor(255,255,255);
        case FieldLines:
            return QColor(100,100,100);
        case wmLeftLeg:
            return QColor(0,0,200);
        case wmRightLeg:
            return QColor(200,0,0);
        case wmBall:
            return QColor(255,102,0);
        case GoalEdgeLinesStart:
            return QColor(0,255,255);
        case GoalEdgeLinesEnd:
            return QColor(255,0,255);
        default:
            return QColor(255,255,255);
        }
    }
};

#endif // GLDISPLAY_H
