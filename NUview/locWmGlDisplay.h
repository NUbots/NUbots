#ifndef LOCWMGLDISPLAY_H
#define LOCWMGLDISPLAY_H

#include <QGLWidget>

class KF;
class Localisation;

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
    };
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

        void drawGoal(QColor colour, float x, float y, float facing);
        void drawBall(QColor colour, float x, float y);
        void drawRobot(QColor colour, float x, float y, float theta);
        void DrawSigmaPoint(QColor colour, float x, float y, float theta);

        void DrawModel(const KF& model);
        void DrawLocalisation(const Localisation& localisation);

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

        bool light;
        bool perspective;
        bool drawRobotModel;
        bool drawSigmaPoints;

};

#endif // LOCWMGLDISPLAY_H
