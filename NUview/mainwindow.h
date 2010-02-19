#ifndef MAINWINDOW_H
#define MAINWINDOW_H


#include <QtGui/QMainWindow>
#include "classificationwidget.h"
#include "connectionwidget.h"
#include "Tools/Image/NUimage.h"
#include "virtualNubot.h"
#include "GLDisplay.h"
#include "openglmanager.h"
#include "locWmGlDisplay.h"
#include "localisationwidget.h"

class QMdiArea;
class LayerSelectionWidget;
class WalkParameterWidget;

namespace Ui
{
    class MainWindow;
}

/**
 * @class  MainWindow
 * @author Aaron Wong
 * @author Steven Nicklin
 *
 * Main window which contains all the widgets.
 */

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
     /** Constructor for MainWindow Class
       *  @param parent The pointer QWidget that is the parent widget for this class.
       */
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

public slots:
    void open();                    //!< To open a file
    void copy();                    //!< To copy the contents of the selected display to file.
    void openLUT();                 //!< To open a LUT file
    void firstFrame();              //!< Takes you back to first frame
    void previousFrame();           //!< Takes you back to previous frame
    void selectFrame();             //!< Takes you to a selected frame
    void nextFrame();               //!< Takes you to next frame
    void lastFrame();               //!< Takes you to last frame
    void cascade();                 //!< Cascades all widgets
    void tile();                    //!< tiles all widgets

    void shrinkToNativeAspectRatio();

    /*!
      @brief Used to select the colour at a given position in the image and
      set it as the selected colour in the classification widget.
      @param x The x coordinate of the pixel.
      @param y The y coordinate of the pixel.
      */
    void SelectColourAtPixel(int x, int y);
    /*!
      @brief Adds the currently selected colours to the lookup table.
      */
    void ClassifySelectedColour();
    /*!
      @brief Selects the central colour to the colour at the given coordinates,
      then add selection to the lookup table.
      @param x The x coordinate of the pixel.
      @param y The y coordinate of the pixel.
      */
    void SelectAndClassifySelectedPixel(int x, int y);
    /*!
      @brief Updates the selected colours.
      */
    void updateSelection();


private:
    //! Virtual robot, does data storage and robot based processing.
    virtualNUbot virtualRobot;
    //! Converts robot formatted data into opengl drawing instructions to form displays.
    OpenglManager glManager;

    //File Opperation Variables
    /*!
      @brief Load the given frame.
      @param Number of the frame to load.
      */
    void LoadFrame(int frameNumber);

    // Initialisation functions
    void createActions();           //!< Generate Actions
    void createMenus();             //!< Generate Menus
    void createContextMenu();       //!< Generate Context Menus
    void createToolBars();          //!< Generate Toolbars
    void createStatusBar();         //!< Generate Status Bars

    ClassificationWidget* classification;       //!< Instance of the classification widget
    ConnectionWidget* connection;               //!< Instance of the connection widget; allows connections with robots
    GLDisplay* imageDisplay;                    //!< Raw Image display.
    GLDisplay* classDisplay;                    //!< Classified Image display
    GLDisplay* horizonDisplay;                  //!< Horizon Line display
    GLDisplay* miscDisplay;                     //!< Misc display
    LocalisationWidget* localisation;           //!< Instance of the localisation widget.

    locWmGlDisplay* wmDisplay;

    LayerSelectionWidget* layerSelection;
    QDockWidget* layerSelectionDock;
    WalkParameterWidget* walkParameter;         //!< A very simple widget to tune the walk parameter
    QDockWidget* walkParameterDock;

    QStatusBar* statusBar;          //!< Instance of the status bar.
    QMdiArea* mdiArea;              //!< Instance of QMdiArea: the main are in the middle of the app (focal point)

    QMenu *fileMenu;                //!< Instance of the file menu
    QMenu *editMenu;                //!< Instance of the edit menu
    QMenu *navigationMenu;          //!< Instance of the naivigation menu
    QMenu *windowMenu;              //!< Instance of the window menu
    QMenu *visionWindowMenu;        //!< Instance of the vision window menu
    QMenu *networkWindowMenu;        //!< Instance of the network window menu


    QToolBar *fileToolBar;          //!< Instance of the file toolbar
    QToolBar *editToolBar;          //!< Instance of the edit toolbar
    QToolBar *navigationToolbar;    //!< Instance of the navigation toolbar
    QToolBar *windowDisplayToolbar; //!< Instance of the window display toolbar


    QAction *openAction;            //!< Instance of the open action
    QAction *copyAction;            //!< Instance of the copy action
    QAction *undoAction;            //!< Instance of the undo action
    QAction *LUT_Action;            //!< Instance of the open action
    QAction *exitAction;            //!< Instance of the exit action
    QAction *firstFrameAction;      //!< Instance of the first frame action; brings you back to first frame
    QAction *previousFrameAction;   //!< Instance of the previous frame action
    QAction *selectFrameAction;     //!< Instance of the select frame action
    QAction *nextFrameAction;       //!< Instance of the next frame action
    QAction *lastFrameAction;       //!< Instance of the last frame action
    QAction *cascadeAction;         //!< Instance of the cascade window action
    QAction *tileAction;            //!< Instance of the tile window action
    QAction *nativeAspectAction;    //!< Instance of the Native Aspect Ratio Action

    int currentFrameNumber;         //!< Variable for current frame in a file
    int totalFrameNumber;                //!< Total frames in file
    QString fileName;               //!< Name of current file loaded
};

#endif // MAINWINDOW_H
