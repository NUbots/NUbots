#ifndef LOCWMGLDISPLAY_H
#define LOCWMGLDISPLAY_H

#include <QGLWidget>

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

protected:
        void mousePressEvent ( QMouseEvent * event );
        void mouseMoveEvent ( QMouseEvent * event );
        void initializeGL();
        void paintGL();
        void resizeGL(int width, int height);
        void drawField();
        void drawGoal(QColor colour, float x, float y, float facing);
        void drawBall(QColor colour, float x, float y);

        GLuint texture;
        GLuint grassTexture;
        GLUquadric* quadratic;
        float viewTranslation[3];
        float viewOrientation[3];
        QPoint dragStartPosition;
        QPoint prevDragPos;
};

#endif // LOCWMGLDISPLAY_H
