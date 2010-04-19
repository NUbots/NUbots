#ifndef MAINWINDOW_H
#define MAINWINDOW_H


#include <QtGui/QMainWindow>
#include "classificationwidget.h"
#include "connectionwidget.h"
#include "Tools/Image/NUimage.h"
#include "virtualnubot.h"
#include "GLDisplay.h"
#include "openglmanager.h"
#include "locWmGlDisplay.h"
#include "localisationwidget.h"
#include "LogFileReader.h"
#include "visionstreamwidget.h"
#include <QHostInfo>

class QMdiArea;
class QMdiSubWindow;
class LayerSelectionWidget;
class WalkParameterWidget;
class KickWidget;
class QTabsWidget;
class cameraSettingsWidget;

class NUviewIO;
class BonjourServiceResolver;


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
    void openLog();                    //!< To open a file
    void openLog(const QString& fileName); //!< To open a log file
    void copy();                    //!< To copy the contents of the selected display to file.
    void openLUT();                 //!< To open a LUT file
    void selectFrame();             //!< Takes you to a selected frame

    void shrinkToNativeAspectRatio();
    void filenameChanged(QString filename); //!< New filename
    void fileClosed(); //!< File was closed.

    void BonjourTest();

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

    void imageFrameChanged(int currFrame, int totalFrames);
protected slots:
//    GLDisplay* createGLDisplay();
//    locWmGlDisplay* createLocWmGlDisplay();
    QMdiSubWindow* createGLDisplay();
    QMdiSubWindow* createLocWmGlDisplay();
    void PrintConnectionInfo(const QHostInfo &hostInfo, int);

private:
    //! Virtual robot, does data storage and robot based processing.
    virtualNUbot virtualRobot;
    NUviewIO* m_nuview_io;
    //! Converts robot formatted data into opengl drawing instructions to form displays.
    OpenglManager glManager;

    int getNumMdiWindowType(const QString& windowType);

    // Initialisation functions
    void createActions();           //!< Generate Actions
    void createMenus();             //!< Generate Menus
    void createContextMenu();       //!< Generate Context Menus
    void createToolBars();          //!< Generate Toolbars
    void createStatusBar();         //!< Generate Status Bars
    void createConnections();       //!< Make required connections

    void readSettings();
    void writeSettings();
    QString getMdiWindowType(QWidget* theWidget);

    ClassificationWidget* classification;       //!< Instance of the classification widget
    ConnectionWidget* connection;               //!< Instance of the connection widget; allows connections with robots
    LocalisationWidget* localisation;           //!< Instance of the localisation widget.

    LayerSelectionWidget* layerSelection;
    visionStreamWidget* VisionStreamer;         //!< Instance of VisionStreamWidget

    //QDockWidget* layerSelectionDock;
    QDockWidget* visionTabDock;
    QDockWidget* networkTabDock;
    WalkParameterWidget* walkParameter;         //!< A very simple widget to tune the walk parameter
    KickWidget* kick;
    cameraSettingsWidget* cameraSetting;
    //QDockWidget* walkParameterDock;

    QStatusBar* statusBar;          //!< Instance of the status bar.
    QMdiArea* mdiArea;              //!< Instance of QMdiArea: the main are in the middle of the app (focal point)
    QTabWidget* visionTabs;
    QTabWidget* networkTabs;

    QMenu *fileMenu;                //!< Instance of the file menu
    QMenu *editMenu;                //!< Instance of the edit menu
    QMenu *navigationMenu;          //!< Instance of the naivigation menu
    QMenu *windowMenu;              //!< Instance of the window menu
    QMenu *visionWindowMenu;        //!< Instance of the vision window menu
    QMenu *localisationWindowMenu;  //!< Instance of the localisation window menu
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
    QAction *newVisionDisplayAction;//!< Instance of the new vision display action.
    QAction *newLocWMDisplayAction;//!< Instance of the new vision display action.

    QAction *doBonjourTestAction;    //!< Instance of the do test Action
    BonjourServiceResolver* bonjourResolver;

    LogFileReader LogReader;

protected:
    void closeEvent(QCloseEvent *event);
};

#endif // MAINWINDOW_H
