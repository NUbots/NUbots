#ifndef LOCWMGLDISPLAY_H
#define LOCWMGLDISPLAY_H

#include <QGLWidget>

class KF;
class Localisation;
class FieldObjects;
class Object;
class StationaryObject;
class NUSensorsData;
class SelfLocalisation;

#include "Localisation/SelfLocalisation.h"

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

protected:
        void keyPressEvent ( QKeyEvent * e );
        void mousePressEvent ( QMouseEvent * event );
        void mouseMoveEvent ( QMouseEvent * event );
        void wheelEvent ( QWheelEvent * event );
        void initializeGL();
        void paintGL();
        void resizeGL(int width, int height);
        void drawField();

        bool loadTexture(QString fileName, GLuint* textureId);

        void drawMarkers();
        void drawObjects();
        void drawOverlays();

        void drawGoal(QColor colour, float x, float y, float facing);
        void drawBall(QColor colour, float x, float y);
        void drawBallMarker(QColor markerColour, float x, float y);
        void drawRobot(QColor colour, float x, float y, float theta);
        void drawRobotMarker(QColor colour, float x, float y, float theta);
        void DrawSigmaPoint(QColor colour, float x, float y, float theta);
        void DrawBallSigma(QColor colour, float x, float y);

        void DrawModelObjects(const KF& model, QColor& modelColor);
        void DrawLocalisationObjects(const Localisation& localisation, QColor& modelColor);

        void DrawModelMarkers(const KF& model, QColor& modelColor);
        void DrawModelMarkers(const Model* model, QColor& modelColor);
        void DrawLocalisationMarkers(const Localisation& localisation, QColor& modelColor);
        void drawLocalisationMarkers(const SelfLocalisation& localisation, QColor& modelColor);

        void DrawLocalisationOverlay(const Localisation& localisation, QColor& modelColor);

        void drawStationaryObjectLabel(const StationaryObject& object);
        void drawFieldObjectLabels(const FieldObjects& theFieldObjectsobject);

        GLuint robotAuraTexture;
        GLuint fieldLineTexture;
        GLuint grassTexture;
        GLuint robotTexture;
        GLuint robotBackTexture;
        GLUquadric* quadratic;
        float viewTranslation[3];
        float viewOrientation[3];
        QPoint dragStartPosition;
        QPoint prevDragPos;

        const Localisation* currentLocalisation;
        const Localisation* localLocalisation;
        const FieldObjects* currentObjects;
        const SelfLocalisation* m_self_loc;
        NUSensorsData* currentSensorData;

        bool light;
        bool perspective;
        bool drawRobotModel;
        bool drawSigmaPoints;
        bool drawBestModelOnly;

};

#endif // LOCWMGLDISPLAY_H
