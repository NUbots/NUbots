#ifndef LUTGLDISPLAY_H
#define LUTGLDISPLAY_H

#include "Vision/VisionTools/classificationcolours.h"
#include <QGLWidget>
#include "openglmanager.h"

class GLUquadric;

class LUTGlDisplay : public QGLWidget
{
Q_OBJECT
public:
    LUTGlDisplay(QWidget *parent, const OpenglManager * shareWidget);
    ~LUTGlDisplay();

    //! Returns the minimum desired size for the window
    QSize minimumSizeHint() const;
    //! Returns the most desired size for the window
    QSize sizeHint() const;
    void restoreState(const QByteArray & state);
    QByteArray saveState() const;

public slots:
    void SetLUT(unsigned char* LUT)
    {
        currentLUT = LUT;
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

        void DrawAxies();
        void DrawLUT(unsigned char* currentLUT);
	void DrawPoint(QColor colour, float x, float y, float z);

        GLUquadric* quadratic;
        float viewTranslation[3];
        float viewOrientation[3];
        QPoint dragStartPosition;
        QPoint prevDragPos;

        unsigned char* currentLUT;

        bool light;
        bool perspective;
        bool trueColours;
        //bool drawRobotModel;
        //bool drawSigmaPoints;

};

#endif // LUTGLDISPLAY_H
