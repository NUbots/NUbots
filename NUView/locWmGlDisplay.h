#ifndef LOCWMGLDISPLAY_H
#define LOCWMGLDISPLAY_H

#include <QGLWidget>
#include "Localisation/SelfLocalisation.h"

class KF;
class Localisation;
class FieldObjects;
class Object;
class StationaryObject;
class NUSensorsData;
class SelfLocalisation;
class GLUquadric;
class Matrix;
class QPoint;
class IKalmanFilter;

struct FieldPose
{
    float x;
    float y;
    float angle;
};

class locWmGlDisplay : public QGLWidget
{
Q_OBJECT
public:
    locWmGlDisplay(QWidget *parent);
    ~locWmGlDisplay();

    //! Returns the minimum desired size for the window
    QSize minimumSizeHint() const;
    //! Returns the most desired size for the window
    QSize sizeHint() const;
    void restoreState(const QByteArray & state);
    QByteArray saveState() const;

public slots:
    void SetLocalisation(const Localisation* newLocalisation)
    {
        currentLocalisation = newLocalisation;
        update();
    }
    void SetLocalLocalisation(const Localisation* newLocalisation)
    {
        localLocalisation = newLocalisation;
        update();
    }

    void setSelfLocalisation(const SelfLocalisation* newSelfLoc)
    {
        m_self_loc = newSelfLoc;
        update();
    }

    void setFieldObjects(const FieldObjects* newFieldObjects)
    {
        currentObjects = newFieldObjects;
        update();
    }
    void setSensorData(NUSensorsData* newSensorData)
    {
        currentSensorData = newSensorData;
        update();
    }
    /*!
      @brief Copy the current image displayed to the system clipboard.
      */
    void snapshotToClipboard();

public slots:
    void setXRotation(int angle);
    void setYRotation(int angle);
    void setZRotation(int angle);

protected:
        void keyPressEvent ( QKeyEvent * e );
        void mousePressEvent ( QMouseEvent * event );
        void mouseMoveEvent ( QMouseEvent * event );
        void wheelEvent ( QWheelEvent * event );
        void initializeGL();
        void paintEvent(QPaintEvent *event);
        //void paintGL(QPaintEvent *event);
        void resizeGL(int width, int height);
        void setupViewport(int width, int height);


        void drawField();
        void drawFieldObjects();

        bool loadTexture(QString fileName, GLuint* textureId);

        void drawMarkers();
        void drawObjects();
        void drawOverlays();

        void drawTriColourBeacon(const QColor& bottomColour, const QColor& middleColour, const QColor& topColour, float x, float y);
        void drawGoal(QColor colour, float x, float y, float facing);
        void drawBall(QColor colour, float x, float y);
        void drawBallMarker(QColor markerColour, float x, float y);
        void drawRobot(QColor colour, float x, float y, float theta);
        void drawRobotMarker(QColor colour, float x, float y, float theta);
        void DrawSigmaPoint(QColor colour, float x, float y, float theta);
        void DrawBallSigma(QColor colour, float x, float y);

        //void DrawErrorElipse(QColor colour, float x, float y);

        FieldPose CalculateErrorElipse(float xx, float xy, float yy);
        void DrawElipse(const QPoint& location, const QPoint& size, float angle, const QColor& lineColour, const QColor& fillColour);

        void DrawModelObjects(const KF& model, const QColor& modelColor);
        void DrawModelObjects(const SelfModel& model, const IKalmanFilter& ball_model, const QColor& modelColor);
        void DrawLocalisationObjects(const Localisation& localisation, const QColor& modelColor);
        void DrawLocalisationObjects(const SelfLocalisation& localisation, const QColor& modelColor);

        void DrawModelMarkers(const KF& model,const QColor& modelColor);
        void DrawModelMarkers(const SelfModel* model, const QColor& modelColor);
        void DrawLocalisationMarkers(const Localisation& localisation, const QColor& modelColor);
        void drawLocalisationMarkers(const SelfLocalisation& localisation, const QColor& modelColor);

        void DrawLocalisationOverlay(const Localisation& localisation, const QColor& modelColor);

        void drawStationaryObjectLabel(const StationaryObject& object);
        void drawFieldObjectLabels(const FieldObjects& theFieldObjectsobject);

        void drawLegend(QPainter* painter);

        FieldPose calculateBallPosition(const SelfModel& robot_model, const Moment& ball_estimate);

        GLuint robotAuraTexture;
        GLuint fieldLineTexture;
        GLuint grassTexture;
        GLuint robotTexture;
        GLuint robotBackTexture;
        float viewTranslation[3];
        int xRot;
        int yRot;
        int zRot;
        QPoint lastPos;

        const Localisation* currentLocalisation;
        const Localisation* localLocalisation;
        const FieldObjects* currentObjects;
        const SelfLocalisation* m_self_loc;
        NUSensorsData* currentSensorData;

        QColor m_currentColour;
        QColor m_localColour;
        QColor m_selfColour;
        QColor m_fieldObjColour;
        QColor m_sensorColour;
        QColor m_backgroundColour;


        bool light;
        bool perspective;
        bool drawRobotModel;
        bool drawSigmaPoints;
        bool drawBestModelOnly;
        bool m_showBall;

};

#endif // LOCWMGLDISPLAY_H
