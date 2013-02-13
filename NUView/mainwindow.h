#ifndef MAINWINDOW_H
#define MAINWINDOW_H


#include <QtGui/QMainWindow>
#include "classificationwidget.h"
#include "connectionwidget.h"
#include "Infrastructure/NUImage/NUImage.h"
#include "virtualnubot.h"
#include "GLDisplay.h"
#include "openglmanager.h"
#include "locWmGlDisplay.h"
#include "LUTGlDisplay.h"
#include "localisationwidget.h"
#include "FileAccess/LogFileReader.h"
#include "visionstreamwidget.h"
#include "locwmstreamwidget.h"
#include "SensorDisplayWidget.h"
#include "ObjectDisplayWidget.h"
#include "GameInformationDisplayWidget.h"
#include "TeamInformationDisplayWidget.h"
#include <QHostInfo>
#include <QList>

class QMdiArea;
class QMdiSubWindow;
#ifndef WIN32
    class ConnectionManager;
#endif
class LayerSelectionWidget;
class WalkParameterWidget;
class KickWidget;
class QTabsWidget;
class cameraSettingsWidget;
class frameInformationWidget;

class NUPlatform;
class NUBlackboard;

class NUViewIO;
class OfflineLocalisationDialog;


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
    void saveViewImage();               //!< To save the contents of the selected display to the local executable directory.
    void openLUT();                 //!< To open a LUT file
    void selectFrame();             //!< Takes you to a selected frame

    void shrinkToNativeAspectRatio();
    void filenameChanged(QString filename); //!< New filename
    void fileClosed(); //!< File was closed.

    void BonjourTest();

    void RunOfflineLocalisation();

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
    QMdiSubWindow* createGLDisplay();
    QMdiSubWindow* createPlotDisplay();
    QMdiSubWindow* createLocWmGlDisplay();
    QMdiSubWindow* createLUTGlDisplay();
    void PrintConnectionInfo(const QHostInfo &hostInfo, int);

private:
    enum ColourScheme
    {
        DefaultColours,
        StevenColours
    };

    //! Virtual robot, does data storage and robot based processing.
    virtualNUbot* virtualRobot;
    NUPlatform* m_platform;
    NUBlackboard* m_blackboard;
    NUViewIO* m_nuview_io;
    //! Converts robot formatted data into opengl drawing instructions to form displays.
    OpenglManager glManager;
    QString m_previous_log_path;

    int getNumMdiWindowType(const QString& windowType);
    void addAsDockable(QWidget* widget, const QString& name);

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

    void setColourTheme(ColourScheme newColors);

    ClassificationWidget* classification;       //!< Instance of the classification widget
    ConnectionWidget* connection;               //!< Instance of the connection widget; allows connections with robots
    LocalisationWidget* localisation;           //!< Instance of the localisation widget.

    LayerSelectionWidget* layerSelection;
    visionStreamWidget* VisionStreamer;         //!< Instance of VisionStreamWidget
    locwmStreamWidget* LocWmStreamer;
    ConnectionManager* m_connection_manager;

    QDockWidget* visionTabDock;
    QDockWidget* networkTabDock;
    WalkParameterWidget* walkParameter;         //!< A very simple widget to tune the walk parameter
    KickWidget* kick;
    cameraSettingsWidget* cameraSetting;
    frameInformationWidget* frameInfo;
    SensorDisplayWidget* sensorDisplay;
    ObjectDisplayWidget* objectDisplayVision;
    ObjectDisplayWidget* objectDisplayLog;
    GameInformationDisplayWidget* gameInfoDisplay;
    TeamInformationDisplayWidget* teamInfoDisplay;
    QTextBrowser* locInfoDisplay;
    QTextBrowser* selflocInfoDisplay;

    QList<QDockWidget*> m_dockable_windows;

    QMdiArea* mdiArea;              //!< Instance of QMdiArea: the main are in the middle of the app (focal point)

    QToolBar *fileToolBar;          //!< Instance of the file toolbar
    QToolBar *editToolBar;          //!< Instance of the edit toolbar
    QToolBar *navigationToolbar;    //!< Instance of the navigation toolbar
    QToolBar *windowDisplayToolbar; //!< Instance of the window display toolbar
    QToolBar *connectionToolBar;	//!< Instance of the connection toolbar

    QAction *openAction;            //!< Instance of the open action
    QAction *copyAction;            //!< Instance of the copy action
    QAction *saveAction;            //!< Instance of the save action
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
    QAction *newPlotDisplayAction;  //!< Instance of the new plot display action.
    QAction *newLocWMDisplayAction;//!< Instance of the new vision display action.
    QAction *newLUTDisplayAction;   //!< Instance of new look up table display action.
    QAction *runOfflineLocalisatonAction; //!< Instance of the offline localisation action.

    QAction *doBonjourTestAction;    //!< Instance of the do test Action
    OfflineLocalisationDialog* offlinelocDialog;
    LogFileReader* LogReader;
    ColourScheme currentColourScheme;

protected:
    void closeEvent(QCloseEvent *event);
};

#endif // MAINWINDOW_H
