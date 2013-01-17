#include "locWmGlDisplay.h"
#include <QDebug>
#include <QMouseEvent>
#include <qclipboard.h>
#include <QApplication>
#include "Tools/Math/General.h"
#include "Localisation/Localisation.h"
#include "Localisation/SelfLocalisation.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Tools/Math/Limit.h"

#include "Tools/Math/Filters/MobileObjectModel.h"
#include "Tools/Math/Filters/IKalmanFilter.h"
#include "Tools/Math/Filters/RobotModel.h"

// Apple has to be different...
#if defined(__APPLE__) || defined(MACOSX)
  #include <glu.h>
#else
  #include <GL/glu.h>
#endif

locWmGlDisplay::locWmGlDisplay(QWidget *parent): QGLWidget(QGLFormat(QGL::SampleBuffers), parent), currentLocalisation(0)
{
    setWindowTitle("World Model Visualisation");
    viewTranslation[0] = 0.0f;
    viewTranslation[1] = 0.0f;
    viewTranslation[2] = -700.0f;
    xRot = 0;
    yRot = 0;
    zRot = 0;
    setFocusPolicy(Qt::StrongFocus); // Required to get keyboard events.
    light = true;
    perspective = true;
    drawRobotModel = false;
    drawSigmaPoints = false;
    drawBestModelOnly = false;
    m_showBall = true;
    m_displayEnabled = true;
    setFont(QFont("Helvetica",12,QFont::Bold,false));
    currentLocalisation = 0;
    currentObjects = 0;
    localLocalisation = 0;
    currentSensorData = 0;
    m_self_loc = 0;

    m_currentColour = QColor(0,0,255);    // Log
    m_localColour = QColor(255,255,0);    // Old
    m_selfColour = QColor(142, 56, 142);  // New
    m_fieldObjColour = QColor(255,0,0);   // Robot when recorded
    m_sensorColour = QColor(0,0,0);         // Externally measured
    m_backgroundColour = QColor(0x3f,0x3f,0x3f);

    setAutoFillBackground(false);
    setMinimumSize(200, 200);
}

locWmGlDisplay::~locWmGlDisplay()
{
    makeCurrent();
    return;
}

static void qNormalizeAngle(int &angle)
{
    while (angle < 0)
        angle += 360 * 16;
    while (angle > 360 * 16)
        angle -= 360 * 16;
}

static inline void qSetColor(float colorVec[], QColor c)
{
    colorVec[0] = c.redF();
    colorVec[1] = c.greenF();
    colorVec[2] = c.blueF();
    colorVec[3] = c.alphaF();
}

void locWmGlDisplay::setXRotation(int angle)
{
    qNormalizeAngle(angle);
    if (angle != xRot)
    {
        xRot = angle;
        update();
    }
}

void locWmGlDisplay::setYRotation(int angle)
{
    qNormalizeAngle(angle);
    if (angle != yRot)
    {
        yRot = angle;
        update();
    }
}

void locWmGlDisplay::setZRotation(int angle)
{
    qNormalizeAngle(angle);
    if (angle != zRot)
    {
        zRot = angle;
        update();
    }
}

void locWmGlDisplay::restoreState(const QByteArray & state)
{
    return;
}

QByteArray locWmGlDisplay::saveState() const
{
    return QByteArray();
}

void locWmGlDisplay::keyPressEvent( QKeyEvent * e )
{
    switch (e->key())
    {
        case Qt::Key_B:
            m_showBall = !m_showBall;
            update();
            break;
        case Qt::Key_L:
            light = !light;
            if (!light)				// If Not Light
            {
                glDisable(GL_LIGHTING);		// Disable Lighting
            }
            else					// Otherwise
            {
                glEnable(GL_LIGHTING);		// Enable Lighting
            }
            update();
            break;
        case Qt::Key_P:
            perspective = !perspective;
            glPushMatrix();
            glMatrixMode(GL_PROJECTION);						// Select The Projection Matrix
            glLoadIdentity();							// Reset The Projection Matrix
            if(perspective)
            {
                gluPerspective(45.0f,(GLfloat)this->width()/(GLfloat)this->height(),0.1f,10000.0f);
            }
            else
            {
                glOrtho(-370.0,  370.0, -270.0, 270.0,0.1,10000.0);
            }
            glMatrixMode(GL_MODELVIEW);						// Select The Modelview Matrix
            glLoadIdentity();							// Reset The Modelview Matrix
            glPopMatrix();
            update();
            break;
        case Qt::Key_R:
            viewTranslation[0] = 0.0f;
            viewTranslation[1] = 0.0f;
            viewTranslation[2] = -700.0f;
            xRot = 0;
            yRot = 0;
            zRot = 0;
            update();
            break;
        case Qt::Key_S:
            drawSigmaPoints = !drawSigmaPoints;
            update();
            break;
        case Qt::Key_K:
            drawRobotModel = !drawRobotModel;
            update();
            break;
        case Qt::Key_M:
            drawBestModelOnly = !drawBestModelOnly;
            update();
            break;
        default:
            e->ignore();
            QGLWidget::keyPressEvent(e);
            break;
    }
    return;
}

void locWmGlDisplay::mousePressEvent ( QMouseEvent * event )
{
    lastPos = event->pos();
}

void locWmGlDisplay::mouseMoveEvent ( QMouseEvent * event )
{
    int dx = event->x() - lastPos.x();
    int dy = event->y() - lastPos.y();

    if (event->buttons() & Qt::LeftButton) {
        setXRotation(xRot + 8 * dy);
        setYRotation(yRot + 8 * dx);
    } else if (event->buttons() & Qt::RightButton) {
        setXRotation(xRot + 8 * dy);
        setZRotation(zRot + 8 * dx);
    }
    lastPos = event->pos();
}

void locWmGlDisplay::wheelEvent ( QWheelEvent * event )
{
    int magnitude = event->delta();
    float zoomDistance = (float)magnitude / 10.0f;
    viewTranslation[2] += zoomDistance;
    Limit zoomLimit(-9000, -20);
    viewTranslation[2] = zoomLimit.clip(viewTranslation[2]);
    update();
}

bool locWmGlDisplay::loadTexture(QString fileName, GLuint* textureId)
{
    makeCurrent();
    glEnable(GL_TEXTURE_2D);                                            // Enable Texture Mapping
    QImage image(fileName);
    QImage texture(QGLWidget::convertToGLFormat( image ));
    glGenTextures( 1, textureId);

    // Create Nearest Filtered Texture
    glBindTexture(GL_TEXTURE_2D, *textureId);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, 4, texture.width(), texture.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, texture.bits());
    glDisable(GL_TEXTURE_2D);   // Disable Texture Mapping
    return true;
}

void locWmGlDisplay::initializeGL()
{
    makeCurrent();
    glEnable(GL_MULTISAMPLE);

    loadTexture(":/textures/robotAura.png", &robotAuraTexture);
    loadTexture(":/textures/FieldLines.png", &fieldLineTexture);
    loadTexture(":/textures/grass_texture.jpg", &grassTexture);
    loadTexture(":/textures/PlaceholderRobot.png", &robotTexture);
    loadTexture(":/textures/PlaceholderRobotBack.png", &robotBackTexture);
}

void locWmGlDisplay::paintEvent(QPaintEvent *event)
{
    makeCurrent();
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();

    qglClearColor(m_backgroundColour);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_DEPTH_TEST);
    //glEnable(GL_CULL_FACE);
    glEnable(GL_MULTISAMPLE);
    glEnable(GL_NORMALIZE);

    glClearDepth(1.0f);                         		// Depth Buffer Setup
    glEnable(GL_DEPTH_TEST);					// Enables Depth Testing
    glDepthFunc(GL_LEQUAL);					// The Type Of Depth Test To Do

    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);		// Really Nice Perspective Calculations

    static GLfloat lightPosition0[]= { 0.0, 0.0, 200.0, 1.0 };      // Light Position
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
//    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition0);

    setupViewport(width(), height());

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    glTranslatef(viewTranslation[0],viewTranslation[1],viewTranslation[2]); // Move to centre of field position.
    glRotatef(xRot / 16.0, 1.0, 0.0, 0.0);
    glRotatef(yRot / 16.0, 0.0, 1.0, 0.0);
    glRotatef(zRot / 16.0, 0.0, 0.0, 1.0);

    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition0);

    drawField();        // Draw the Standard Field Layout.
    if(m_displayEnabled)
    {
        drawMarkers();
        drawObjects();
    }

    drawFieldObjects();

    if(m_displayEnabled)
    {
        drawOverlays();
    }

    glShadeModel(GL_FLAT);
    //glDisable(GL_CULL_FACE);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);

    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    if(m_displayEnabled)
    {
        drawLegend(&painter);
    }
    painter.end();
    return;
}

void locWmGlDisplay::setupViewport(int width, int height)
{
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    if(perspective)
        gluPerspective(45.0f,(GLfloat)width/(GLfloat)height,0.1f,10000.0f);
    else
    {
    #ifdef QT_OPENGL_ES
        glOrthof(-370.0,  370.0, -270.0, 270.0,0.1,10000.0);
    #else
        glOrtho(-370.0,  370.0, -270.0, 270.0,0.1,10000.0);
    #endif
    }
    glMatrixMode(GL_MODELVIEW);
}

void locWmGlDisplay::drawMarkers()
{
    if(currentSensorData)
    {
        std::vector<float> gpsPosition;
        float compassHeading;
        if(currentSensorData->getGps(gpsPosition))
        {
            if(!currentSensorData->getCompass(compassHeading)) compassHeading = 3.14;
            drawRobotMarker(m_sensorColour, gpsPosition[0], gpsPosition[1],compassHeading);
        }

    }

    if(currentObjects)
    {
        drawRobotMarker(m_fieldObjColour, currentObjects->self.wmX(), currentObjects->self.wmY(),currentObjects->self.Heading());
    }

    if(currentLocalisation)
    {
        DrawLocalisationMarkers(*currentLocalisation, m_currentColour);
    }
    if(localLocalisation)
    {
        DrawLocalisationMarkers(*localLocalisation, m_localColour);
    }
    if(m_self_loc)
    {
        drawLocalisationMarkers(*m_self_loc, m_selfColour);
    }
}

void locWmGlDisplay::drawObjects()
{
    if(currentSensorData)
    {
        std::vector<float> gpsPosition;
        float compassHeading;
        if(currentSensorData->getGps(gpsPosition))
        {
            if(!currentSensorData->getCompass(compassHeading)) compassHeading = 3.14;
            drawRobot(m_sensorColour, gpsPosition[0], gpsPosition[1],compassHeading);
        }

    }
    if(currentLocalisation)
    {
        DrawLocalisationObjects(*currentLocalisation, m_currentColour);
    }
    if(localLocalisation)
    {
        DrawLocalisationObjects(*localLocalisation, m_localColour);
    }
    if(m_self_loc)
    {
        DrawLocalisationObjects(*m_self_loc, m_selfColour);
    }
}

void locWmGlDisplay::drawOverlays()
{
    if(currentLocalisation)
    {
        DrawLocalisationOverlay(*currentLocalisation, m_currentColour);
    }
    if(currentObjects)
    {
        drawFieldObjectLabels(*currentObjects);
    }
    if(localLocalisation)
    {
        DrawLocalisationOverlay(*localLocalisation, m_localColour);
    }
}

void locWmGlDisplay::snapshotToClipboard()
{
    paintGL(); // Redraw the scene in case it needs to be updated.
    QClipboard *cb = QApplication::clipboard(); // get the clipboard
    QImage tempPicture(this->grabFrameBuffer(false)); // grab current image
    cb->setImage(tempPicture); // put current image on the clipboard.
}

void locWmGlDisplay::drawField()
{
    glPushMatrix();
    glEnable(GL_TEXTURE_2D);                    // Enable Texture Mapping
    glShadeModel(GL_SMOOTH);    		// Enable Smooth Shading
    glEnable(GL_BLEND);		// Turn Blending On
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

    GLfloat faceColor[4];
    qSetColor(faceColor, Qt::white);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, faceColor);

    glEnable(GL_TEXTURE_2D);                                            // Enable Texture Mapping
    glBindTexture(GL_TEXTURE_2D, grassTexture);
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);    // Turn off filtering of textures
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);    // Turn off filtering of textures

        const unsigned int x_size = 10;
        const unsigned int y_size = 10;
        glBegin(GL_QUADS);
        // Draw texture using 10x10 grids for shading.
        for (int x = -370; x < 370; x += x_size)
        {
            for (int y = -270; y < 270; y += y_size)
            {
                float texBottom = (y+270) / 100.0f;
                float texTop = texBottom + y_size / 100.0;
                float texLeft = (x+370) / 100.0f;
                float texRight = texLeft + x_size / 100.0;

                float quadBottom = y;
                float quadTop = quadBottom + y_size;
                float quadLeft = x;
                float quadRight = quadLeft + x_size;

                glNormal3f(0.0f,0.0f,1.0f);	// Set The Normal
                glTexCoord2f(texLeft, texBottom); glVertex3f(quadLeft,  quadBottom,  0.0f);      // Bottom Left Of The Texture and Quad
                glTexCoord2f(texRight, texBottom); glVertex3f(quadRight,  quadBottom,  0.0f);    // Bottom Right Of The Texture and Quad
                glTexCoord2f(texRight, texTop); glVertex3f(quadRight, quadTop,  0.0f);     // Top Right Of The Texture and Quad
                glTexCoord2f(texLeft, texTop); glVertex3f(quadLeft, quadTop,  0.0f);       // Top Left Of The Texture and Quad
            }
        }
        glEnd();

    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, faceColor);
    glBindTexture(GL_TEXTURE_2D, fieldLineTexture);
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);    // Turn off filtering of textures
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);    // Turn off filtering of textures

        glBegin(GL_QUADS);
        // Draw texture using 10x10 grids for shading.
        for (int x = -370; x < 370; x += x_size)
        {
            for (int y = -270; y < 270; y += y_size)
            {
                float texBottom = (y + 270) / 540.0f;
                float texTop = (y + 270 + y_size) / 540.0f;
                float texLeft = (x+370) / 740.0f;
                float texRight = (x + 370 + x_size) / 740.0f;

                float quadBottom = y;
                float quadTop = quadBottom + y_size;
                float quadLeft = x;
                float quadRight = quadLeft + x_size;

                glNormal3f(0.0f,0.0f,1.0f);	// Set The Normal
                glTexCoord2f(texLeft, texBottom); glVertex3f(quadLeft,  quadBottom,  0.0f);      // Bottom Left Of The Texture and Quad
                glTexCoord2f(texRight, texBottom); glVertex3f(quadRight,  quadBottom,  0.0f);    // Bottom Right Of The Texture and Quad
                glTexCoord2f(texRight, texTop); glVertex3f(quadRight, quadTop,  0.0f);     // Top Right Of The Texture and Quad
                glTexCoord2f(texLeft, texTop); glVertex3f(quadLeft, quadTop,  0.0f);       // Top Left Of The Texture and Quad
            }
        }
        glEnd();
    glDisable(GL_BLEND);            // Turn Blending Off
    glDisable(GL_TEXTURE_2D);       // Disable Texture Mapping
    glPopMatrix();
}

void locWmGlDisplay::drawFieldObjects()
{
    glPushMatrix();
    drawGoal(Qt::blue,-300,0.0,0.0);
    drawGoal(Qt::yellow,300,0.0,180.0);
    drawTriColourBeacon(Qt::yellow, Qt::blue, Qt::yellow, 0.f, 240.f);
    drawTriColourBeacon(Qt::blue, Qt::yellow, Qt::blue, 0.f, -240.f);
    glPopMatrix();
}

void locWmGlDisplay::drawTriColourBeacon(const QColor& bottomColour, const QColor& middleColour, const QColor& topColour, float x, float y)
{
    const float postRadius  = 5.f;
    const float segmentHeight = 15.f;
    const int slices = 64;
    const int stacks = 64;
    const int loops = 64;

    glPushMatrix();
    GLfloat currentColour[4];

    glShadeModel(GL_SMOOTH);    		// Enable Smooth Shading
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

    GLUquadric* quad = gluNewQuadric();
    gluQuadricDrawStyle(quad, GLU_FILL);
    gluQuadricNormals(quad, GLU_SMOOTH);	// Create Smooth Normals

    glTranslatef(x,y,0.0f);    // Move to centre of beacon.
    qSetColor(currentColour, bottomColour);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, currentColour);
    gluCylinder(quad,postRadius,postRadius,segmentHeight,slices,stacks);	// Draw lower segment

    glTranslatef(0,0,segmentHeight);    // Move up to next segment.
    qSetColor(currentColour, middleColour);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, currentColour);
    gluCylinder(quad,postRadius,postRadius,segmentHeight,slices,stacks);	// Draw middle segment

    glTranslatef(0,0,segmentHeight);    // Move up to next segment.
    qSetColor(currentColour, topColour);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, currentColour);
    gluCylinder(quad,postRadius,postRadius,segmentHeight,slices,stacks);	// Draw top segment

    // Draw to top cap
    glTranslatef(0,0,segmentHeight);
    gluDisk(quad, 0.0, postRadius, slices, loops);

    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);
    glPopMatrix();
}

void locWmGlDisplay::drawGoal(QColor colour, float x, float y, float facing)
{
    const float postRadius = 5.f;
    const float crossBarRadius = 2.5f;
    const float postHeight = 80.f;
    const int slices = 64;
    const int stacks = 64;
    const int loops = 64;
    glPushMatrix();
    GLfloat goalColour[4];
    qSetColor(goalColour, colour);
    GLfloat white[4];
    QColor temp_white = Qt::white;
    temp_white.setAlphaF(colour.alphaF());
    qSetColor(white, temp_white);

    glShadeModel(GL_SMOOTH);    		// Enable Smooth Shading
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

    GLUquadric* quad = gluNewQuadric();
    gluQuadricDrawStyle(quad, GLU_FILL);
    gluQuadricNormals(quad, GLU_SMOOTH);	// Create Smooth Normals

    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, goalColour);
    glTranslatef(x,y,0.0f);    // Move to centre of goal.
    glRotatef(facing,0.0f,0.0f,1.0f);				// Rotate The Pyramid On It's Y Axis

    glTranslatef(0.0f,70.0f,0.0f);    // Move to right post.
    gluCylinder(quad,postRadius,postRadius,postHeight,slices,stacks);	// Draw right post
    glTranslatef(0.0f,0.0f,postHeight);    // Move to top of post.
    gluDisk(quad,0.f,postRadius,slices,loops);	// Draw cap
    glTranslatef(0.0f,0.0f,-postHeight);    // Move back down


    // Draw right post triangle.
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, white);
    glBegin(GL_TRIANGLES);				// Drawing Using Triangles
        glVertex3f( -postRadius, 0.0f, 40.0f);		// Top
        glVertex3f( -postRadius,0.0f, 0.0f);		// Bottom Right
        glVertex3f(-postRadius - 40.0f,0.0f, 0.0f);	// Bottom Left
    glEnd();						// Finished Drawing The Triangle


    glTranslatef(0.0f,-2*70.0f,0.0f);    // Move to left post.
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, goalColour);
    gluCylinder(quad,postRadius,postRadius,postHeight,slices,stacks);	// Draw left post
    glTranslatef(0.0f,0.0f,postHeight);    // Move to top of post.
    gluDisk(quad,0.f,postRadius,slices,loops);	// Draw cap
    glTranslatef(0.0f,0.0f,-postHeight);    // Move back down.

    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, white);
    glBegin(GL_TRIANGLES);				// Drawing Using Triangles
        glVertex3f( -postRadius, 0.0f, 40.0f);		// Top
        glVertex3f( -postRadius,0.0f, 0.0f);		// Bottom Right
        glVertex3f(-postRadius - 40.0f,0.0f, 0.0f);	// Bottom Left
    glEnd();						// Finished Drawing The Triangle

    glTranslatef(0.0f,0.f, postHeight - crossBarRadius - 1);    // Move to top of left post.
    glRotatef(-90.0,1.0f,0.0f,0.0f);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, goalColour);
    gluCylinder(quad,crossBarRadius,crossBarRadius,140.0f,slices,stacks);	// Draw cross bar

    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);
    glPopMatrix();

}

void locWmGlDisplay::drawBall(QColor colour, float x, float y)
{
    const float ballRadius = 6.5/2.0;
    GLfloat ballColour[4];
    qSetColor(ballColour, colour);

    GLUquadric* quad = gluNewQuadric();
    gluQuadricDrawStyle(quad, GLU_FILL);
    gluQuadricNormals(quad, GLU_SMOOTH);	// Create Smooth Normals

    glPushMatrix();
    glEnable(GL_BLEND);		// Turn Blending On
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, ballColour);
    glTranslatef(x,y,ballRadius);    // Move to centre of ball.
    gluSphere(quad,ballRadius,128,128);		// Draw A Sphere
    glColor4f(1.0f,1.0f,1.0f,1.0f); // Back to white
    glDisable(GL_BLEND);		// Turn Blending On
    glPopMatrix();
}

void locWmGlDisplay::drawBallMarker(QColor colour, float x, float y)
{
    const float ballRadius = 6.5/2.0;

    GLUquadric* quad = gluNewQuadric();
    gluQuadricDrawStyle(quad, GLU_FILL);
    gluQuadricNormals(quad, GLU_SMOOTH);	// Create Smooth Normals

    glPushMatrix();
    glEnable(GL_BLEND);		// Turn Blending On
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_DEPTH_TEST);		// Turn Z Buffer testing Off
    glDisable(GL_LIGHTING);      // Disable Global Lighting
    glColor4ub(colour.red(),colour.green(),colour.blue(),colour.alpha());
    glTranslatef(x,y,0);    // Move to centre of ball.
    gluDisk(quad, ballRadius, ballRadius+2, 32, 32);
    glEnable(GL_DEPTH_TEST);		// Turn Z Buffer testing On
    glEnable(GL_LIGHTING);      // Enable Global Lighting
    glDisable(GL_BLEND);		// Turn Blending On
    glPopMatrix();
    return;
}

void locWmGlDisplay::drawRobotMarker(QColor colour, float x, float y, float theta)
{
    const float robotWidth = 30.0f;

    GLUquadric* quad = gluNewQuadric();
    gluQuadricDrawStyle(quad, GLU_FILL);
    gluQuadricNormals(quad, GLU_SMOOTH);	// Create Smooth Normals

    glPushMatrix();
    glEnable(GL_BLEND);		// Turn Blending On
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_TEXTURE_2D);       // Enable Texture Mapping
    glTranslatef(x,y,0);    // Move to centre of robot.
    glRotatef(mathGeneral::rad2deg(theta),0.0f, 0.0f, 1.0f);

    glDisable(GL_DEPTH_TEST);		// Turn Z Buffer testing Off
    glDisable(GL_LIGHTING);      // Disable Global Lighting
    // Draw Aura
    glColor4ub(colour.red(),colour.green(),colour.blue(),colour.alpha());
    glBindTexture(GL_TEXTURE_2D, robotAuraTexture);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);    // Turn off filtering of textures
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);    // Turn off filtering of textures
    glBegin(GL_QUADS);
        glNormal3f(0.0f,0.0f,1.0f);	// Set The Normal
        glTexCoord2f(0.0f, 0.0f); glVertex3f(-robotWidth/2.0f,  -robotWidth/2.0f*1.2,  1.5);      // Bottom Left Of The Texture and Quad
        glTexCoord2f(1.0f, 0.0f); glVertex3f(-robotWidth/2.0f, robotWidth/2.0f*1.2,  1.5);    // Bottom Right Of The Texture and Quad
        glTexCoord2f(1.0f, 1.0f); glVertex3f(+robotWidth/2.0f, robotWidth/2.0f*1.2,  1.5);     // Top Right Of The Texture and Quad
        glTexCoord2f(0.0f, 1.0f); glVertex3f(+robotWidth/2.0f, -robotWidth/2.0f*1.2,  1.5);       // Top Left Of The Texture and Quad
    glEnd();
    glEnable(GL_DEPTH_TEST);		// Turn Z Buffer testing On

    glDisable(GL_TEXTURE_2D);       // Disable Texture Mapping
    glDisable(GL_BLEND);		// Turn Blending Off
    glPopMatrix();
}

void locWmGlDisplay::drawRobot(QColor colour, float x, float y, float theta)
{
    const float robotHeight = 58.0f;
    glEnable(GL_LIGHTING);      // Enable Global Lighting
    const float robotWidth = 30.0f;
    glPushMatrix();
    glEnable(GL_BLEND);		// Turn Blending On
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_TEXTURE_2D);       // Enable Texture Mapping
    glTranslatef(x,y,0);    // Move to centre of robot.
    glRotatef(mathGeneral::rad2deg(theta),0.0f, 0.0f, 1.0f);

    GLfloat white[4];
    QColor temp_white = Qt::white;
    temp_white.setAlphaF(colour.alphaF());
    qSetColor(white, temp_white);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, white);

    if(drawRobotModel)
    {
        glTranslatef(0,0,robotHeight/2.0);    // Move to centre of robot.

        // Draw Front
        glBindTexture(GL_TEXTURE_2D, robotTexture);
            glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);    // Turn off filtering of textures
            glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);    // Turn off filtering of textures
            glBegin(GL_QUADS);
                glNormal3f(0.0f,0.0f,1.0f);	// Set The Normal
                glTexCoord2f(0.0f, 0.0f); glVertex3f(0.0f,  -robotWidth/2.0f,  -robotHeight/2.0);      // Bottom Left Of The Texture and Quad
                glTexCoord2f(1.0f, 0.0f); glVertex3f(0.0f, robotWidth/2.0f,  -robotHeight/2.0);    // Bottom Right Of The Texture and Quad
                glTexCoord2f(1.0f, 1.0f); glVertex3f(0.0f, robotWidth/2.0f,  robotHeight/2.0);     // Top Right Of The Texture and Quad
                glTexCoord2f(0.0f, 1.0f); glVertex3f(0.0f, -robotWidth/2.0f,  robotHeight/2.0);       // Top Left Of The Texture and Quad
            glEnd();

        // Draw back
        glBindTexture(GL_TEXTURE_2D, robotBackTexture);
            glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);    // Turn off filtering of textures
            glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);    // Turn off filtering of textures
            glBegin(GL_QUADS);
                glNormal3f(0.0f,0.0f,1.0f);	// Set The Normal
                glTexCoord2f(0.0f, 0.0f); glVertex3f(-0.5f,  robotWidth/2.0f,  -robotHeight/2.0);      // Bottom Left Of The Texture and Quad
                glTexCoord2f(1.0f, 0.0f); glVertex3f(-0.5f, -robotWidth/2.0f,  -robotHeight/2.0);    // Bottom Right Of The Texture and Quad
                glTexCoord2f(1.0f, 1.0f); glVertex3f(-0.5f, -robotWidth/2.0f,  robotHeight/2.0);     // Top Right Of The Texture and Quad
                glTexCoord2f(0.0f, 1.0f); glVertex3f(-0.5f, robotWidth/2.0f,  robotHeight/2.0);       // Top Left Of The Texture and Quad
            glEnd();
    }

    glDisable(GL_TEXTURE_2D);       // Disable Texture Mapping
    glDisable(GL_BLEND);		// Turn Blending Off
    glPopMatrix();
}

void locWmGlDisplay::DrawSigmaPoint(QColor colour, float x, float y, float theta)
{
    const float robotWidth = 15.0f;
    glPushMatrix();
    glEnable(GL_BLEND);		// Turn Blending On
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_TEXTURE_2D);       // Disable Texture Mapping
    glTranslatef(x,y,0);    // Move to centre of robot.
    glRotatef(mathGeneral::rad2deg(theta),0.0f, 0.0f, 1.0f);

    // Draw Aura
    glColor4ub(colour.red(),colour.green(),colour.blue(),colour.alpha());
    glDisable(GL_DEPTH_TEST);		// Turn Z Buffer testing Off
    glDisable(GL_LIGHTING);      // Disable Global Lighting
    glBindTexture(GL_TEXTURE_2D, robotAuraTexture);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);    // Turn off filtering of textures
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);    // Turn off filtering of textures
    glBegin(GL_QUADS);
        glNormal3f(0.0f,0.0f,1.0f);	// Set The Normal
        glTexCoord2f(0.0f, 0.0f); glVertex3f(-robotWidth/2.0f,  -robotWidth/2.0f*1.2,  1.5);      // Bottom Left Of The Texture and Quad
        glTexCoord2f(1.0f, 0.0f); glVertex3f(-robotWidth/2.0f, robotWidth/2.0f*1.2,  1.5);    // Bottom Right Of The Texture and Quad
        glTexCoord2f(1.0f, 1.0f); glVertex3f(+robotWidth/2.0f, robotWidth/2.0f*1.2,  1.5);     // Top Right Of The Texture and Quad
        glTexCoord2f(0.0f, 1.0f); glVertex3f(+robotWidth/2.0f, -robotWidth/2.0f*1.2,  1.5);       // Top Left Of The Texture and Quad
    glEnd();
    glEnable(GL_DEPTH_TEST);		// Turn Z Buffer testing On
    glEnable(GL_LIGHTING);      // Enable Global Lighting
    glDisable(GL_TEXTURE_2D);       // Disable Texture Mapping
    glDisable(GL_BLEND);		// Turn Blending Off
    glPopMatrix();
}

void locWmGlDisplay::DrawBallSigma(QColor colour, float x, float y)
{
    const float ballRadius = 2.0;

    GLUquadric* quad = gluNewQuadric();
    gluQuadricDrawStyle(quad, GLU_FILL);
    gluQuadricNormals(quad, GLU_SMOOTH);	// Create Smooth Normals

    glPushMatrix();
    glEnable(GL_BLEND);		// Turn Blending On
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_DEPTH_TEST);		// Turn Z Buffer testing Off
    glDisable(GL_LIGHTING);      // Disable Global Lighting
    glColor4ub(colour.red(), colour.green(), colour.blue(), colour.alpha());
    glTranslatef(x,y,0);    // Move to centre of ball.
    gluDisk(quad, ballRadius, ballRadius+2, 32, 32);
    glColor4f(1.0f,1.0f,1.0f,1.0f); // Back to white
    glEnable(GL_DEPTH_TEST);		// Turn Z Buffer testing On
    glEnable(GL_LIGHTING);      // Enable Global Lighting
    glDisable(GL_BLEND);		// Turn Blending On
    glPopMatrix();
    return;
}

void locWmGlDisplay::DrawModelObjects(const KF& model, const QColor& modelColor)
{
    QColor drawColor(modelColor);
    int alpha = drawColor.alpha();
    drawRobot(drawColor, model.state(KF::selfX), model.state(KF::selfY), model.state(KF::selfTheta));
    if(drawSigmaPoints)
    {
        Matrix sigmaPoints = model.CalculateSigmaPoints();
        for (int i=1; i < sigmaPoints.getn(); i++)
        {
            DrawSigmaPoint(drawColor, sigmaPoints[KF::selfX][i], sigmaPoints[KF::selfY][i], sigmaPoints[KF::selfTheta][i]);
        }
    }
    if(m_showBall)
    {
        drawBall(QColor(255,165,0,alpha), model.state(KF::ballX), model.state(KF::ballY));
    }
}

void locWmGlDisplay::DrawModelObjects(const Moment& model, const Moment& ball_model, const QColor& modelColor)
{
    QColor drawColor(modelColor);
    int alpha = drawColor.alpha();
    drawRobot(drawColor, model.mean(RobotModel::kstates_x), model.mean(RobotModel::kstates_y), model.mean(RobotModel::kstates_heading));

    if(m_showBall)
    {
        FieldPose ball_pose = calculateBallPosition(model, ball_model);
        drawBall(QColor(255,165,0,alpha), ball_pose.x, ball_pose.y);
    }
}

void locWmGlDisplay::DrawLocalisationObjects(const Localisation& localisation, const QColor& modelColor)
{   
    if(drawBestModelOnly)
    {
        const KF model = localisation.getBestModel();
        DrawModelObjects(model, modelColor);
    }
    else
    {
        for(int modelID = 0; modelID < Localisation::c_MAX_MODELS; modelID++)
        {
            const KF model = localisation.getModel(modelID);
            if(model.active())
            {
                QColor drawColor(modelColor);
                int alpha = 255*model.alpha();
                if(alpha < 25) alpha = 25;
                drawColor.setAlpha(alpha);
                DrawModelObjects(model, drawColor);
            }
        }
    }
}

void locWmGlDisplay::DrawLocalisationObjects(const SelfLocalisation& localisation, const QColor& modelColor)
{
    const IKalmanFilter* ball_model = localisation.getBallModel();
    if(drawBestModelOnly)
    {
        const IKalmanFilter* model = localisation.getBestModel();
        DrawModelObjects(model->estimate(), ball_model->estimate(), modelColor);
        std::cout << model->getFilterWeight() << std::endl;
    }
    else
    {
        std::list<IKalmanFilter*> models = localisation.allModels();

        for(std::list<IKalmanFilter*>::iterator model = models.begin(); model != models.end(); ++model)
        {
            const IKalmanFilter* currModel = (*model);
            if(currModel->active())
            {
                QColor drawColor(modelColor);
                int alpha = 255*currModel->getFilterWeight();
                if(alpha < 25) alpha = 25;
                drawColor.setAlpha(alpha);
                DrawModelObjects(currModel->estimate(), ball_model->estimate(), drawColor);
            }
        }
    }
}

void locWmGlDisplay::DrawModelMarkers(const KF& model, const QColor& modelColor)
{
    const int c_min_display_alpha = 50; // Minimum alpha to use when drawing a model.
    float mean_x = model.state(KF::selfX);
    float mean_y = model.state(KF::selfY);
    float mean_angle = model.state(KF::selfTheta);
    drawRobotMarker(modelColor, mean_x, mean_y, mean_angle);

    // Need to multiply by its transpose to do conversion: standard deviation -> variance.
    Matrix cov = model.stateStandardDeviations * model.stateStandardDeviations.transp();
    float xx = cov[KF::selfX][KF::selfX];
    float xy = cov[KF::selfX][KF::selfY];
    float yy = cov[KF::selfY][KF::selfY];
    FieldPose pose = CalculateErrorElipse(xx,xy,yy);

    QColor outline(modelColor);
    outline.setAlphaF(std::max((float)model.alpha(), c_min_display_alpha / 255.0f));
    QColor fill(modelColor);
    fill.setAlpha(std::max((int)(100 * model.alpha()), c_min_display_alpha));
    DrawElipse(QPoint(mean_x,mean_y), QPoint(pose.x,pose.y), mathGeneral::rad2deg(pose.angle), outline, fill);

    if(drawSigmaPoints)
    {
        Matrix sigmaPoints = model.CalculateSigmaPoints();
        for (int i=1; i < sigmaPoints.getn(); i++)
        {
            if(m_showBall)
            {
                DrawBallSigma(modelColor, sigmaPoints[KF::ballX][i], sigmaPoints[KF::ballY][i]);
            }
            DrawSigmaPoint(modelColor, sigmaPoints[KF::selfX][i], sigmaPoints[KF::selfY][i], sigmaPoints[KF::selfTheta][i]);
        }
    }
    if(m_showBall)
    {
        drawBallMarker(modelColor, model.state(KF::ballX), model.state(KF::ballY));
    }
}

void locWmGlDisplay::DrawModelMarkers(const Moment& model, const QColor& modelColor)
{
    const int c_min_display_alpha = 50; // Minimum alpha to use when drawing a model.
    float mean_x = model.mean(RobotModel::kstates_x);
    float mean_y = model.mean(RobotModel::kstates_y);
    float mean_angle = model.mean(RobotModel::kstates_heading);
    drawRobotMarker(modelColor, mean_x, mean_y, mean_angle);

    Matrix cov = model.covariance();
    float xx = cov[RobotModel::kstates_x][RobotModel::kstates_x];
    float xy = cov[RobotModel::kstates_x][RobotModel::kstates_y];
    float yy = cov[RobotModel::kstates_y][RobotModel::kstates_y];
    FieldPose pose = CalculateErrorElipse(xx,xy,yy);

    QColor outline(modelColor);
    float model_draw_alpha = modelColor.alphaF();
    outline.setAlphaF(std::max(model_draw_alpha, c_min_display_alpha / 255.0f));
    QColor fill(modelColor);
    fill.setAlpha(std::max((int)(100 * model_draw_alpha), c_min_display_alpha));
    DrawElipse(QPoint(mean_x,mean_y), QPoint(pose.x,pose.y), mathGeneral::rad2deg(pose.angle), outline, fill);
}

void locWmGlDisplay::DrawLocalisationMarkers(const Localisation& localisation, const QColor& modelColor)
{
    QColor drawColor(modelColor);
    const int c_min_display_alpha = 50; // Minimum alpha to use when drawing a model.
    if(drawBestModelOnly)
    {
        drawColor.setAlpha(255);
        const KF model = localisation.getBestModel();
        DrawModelMarkers(model, drawColor);
    }
    else
    {
        QString displayString("Model %1 (%2%)");
        for(int modelID = 0; modelID < Localisation::c_MAX_MODELS; modelID++)
        {
            const KF model = localisation.getModel(modelID);
            if(model.active())
            {
                int alpha = std::max(c_min_display_alpha, (int)(255*model.alpha()));
                drawColor.setAlpha(alpha);
                DrawModelMarkers(model, drawColor);
            }
        }
    }
}

void locWmGlDisplay::drawLocalisationMarkers(const SelfLocalisation& localisation, const QColor& modelColor)
{
    QColor drawColor(modelColor);
    const int c_min_display_alpha = 50; // Minimum alpha to use when drawing a model.

    const IKalmanFilter* ball_model = localisation.getBallModel();
    Moment ball_estimate = ball_model->estimate();

    Matrix cov = ball_estimate.covariance();
    float xx = cov[MobileObjectModel::kstates_x_pos][MobileObjectModel::kstates_x_pos];
    float xy = cov[MobileObjectModel::kstates_x_pos][MobileObjectModel::kstates_y_pos];
    float yy = cov[MobileObjectModel::kstates_y_pos][MobileObjectModel::kstates_y_pos];
    FieldPose ball_ellipse = CalculateErrorElipse(xx,xy,yy);

    QColor fill(modelColor);

    if(drawBestModelOnly)
    {
        drawColor.setAlpha(255);
        const IKalmanFilter* model = localisation.getBestModel();
        DrawModelMarkers(model->estimate(), drawColor);
        FieldPose ball_pose = calculateBallPosition(model->estimate(), ball_estimate);
        if(m_showBall)
        {
            drawBallMarker(drawColor, ball_pose.x, ball_pose.y);
            fill.setAlpha(std::max((int)(100), c_min_display_alpha));
            DrawElipse(QPoint(ball_pose.x,ball_pose.y), QPoint(ball_ellipse.x,ball_ellipse.y), mathGeneral::rad2deg(ball_ellipse.angle), drawColor, fill);
        }
    }
    else
    {
        QString displayString("Model %1 (%2%)");
        std::list<IKalmanFilter*> models = localisation.allModels();
        for(std::list<IKalmanFilter*>::const_iterator model_it = models.begin(); model_it != models.end(); ++model_it)
        {
            if((*model_it)->active())
            {
                int alpha = std::max(c_min_display_alpha, (int)(255*(*model_it)->getFilterWeight()));
                drawColor.setAlpha(alpha);
                DrawModelMarkers((*model_it)->estimate(), drawColor);
                if(m_showBall)
                {
                    FieldPose ball_pose = calculateBallPosition((*model_it)->estimate(), ball_estimate);
                    drawBallMarker(drawColor, ball_pose.x, ball_pose.y);
                    fill.setAlpha(std::max((int)(100), c_min_display_alpha));
                    DrawElipse(QPoint(ball_pose.x,ball_pose.y), QPoint(ball_ellipse.x,ball_ellipse.y), mathGeneral::rad2deg(ball_ellipse.angle), drawColor, fill);
                }
            }
        }
    }

}

FieldPose locWmGlDisplay::calculateBallPosition(const Moment &robot_estimate, const Moment &ball_estimate)
{
    FieldPose result;
    float selfX = robot_estimate.mean(RobotModel::kstates_x);
    float selfY = robot_estimate.mean(RobotModel::kstates_y);
    float selfHeading = robot_estimate.mean(RobotModel::kstates_heading);

    // pre-calculate the trig.
    float hcos = cos(selfHeading);
    float hsin = sin(selfHeading);

    float relBallX = ball_estimate.mean(MobileObjectModel::kstates_x_pos);
    float relBallY = ball_estimate.mean(MobileObjectModel::kstates_y_pos);
    // Rotate the relative ball postion to alight with the forward looking robot on the field.
    float rotatedX = relBallX * hcos - relBallY * hsin;
    float rotatedY = relBallX * hsin + relBallY * hcos;

    // Calculate the Ball location in field coordinates.
    result.x = selfX + rotatedX;
    result.y = selfY + rotatedY;
    return result;
}

FieldPose locWmGlDisplay::CalculateErrorElipse(float xx, float xy, float yy)
{
    const float scalefactor = 2.4477; // for 95% confidence.
    FieldPose result;
    float Eig1 = (xx+yy)/2 + sqrt(4*xy*xy + (xx-yy)*(xx-yy))/2;
    float Eig2 = (xx+yy)/2 - sqrt(4*xy*xy + (xx-yy)*(xx-yy))/2;

    float maxEig = std::max(Eig1,Eig2);
    float minEig = std::min(Eig1,Eig2);

    if(sqrt(xx) < sqrt(yy))
    {
        result.x = sqrt(minEig) * scalefactor;
        result.y = sqrt(maxEig) * scalefactor;
    }
    else
    {
        result.x = sqrt(maxEig) * scalefactor;
        result.y = sqrt(minEig) * scalefactor;
    }

    const float aspectratio = 1.0;
    result.angle = 0.5 * atan((1/aspectratio) * (2*xy) / (xx-yy));
    return result;
}

void locWmGlDisplay::DrawElipse(const QPoint& location, const QPoint& size, float angle, const QColor& lineColour, const QColor& fillColour)
{
    const float DEG2RAD = 3.14159/180;
    glLineWidth(3);

    float xradius = size.x();
    float yradius = size.y();

    glDisable(GL_LIGHTING);      // Enable Global Lighting
    glDisable(GL_DEPTH_TEST);		// Turn Z Buffer testing Off
    glEnable(GL_BLEND);		// Turn Blending On
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

    glPushMatrix();
    glTranslatef(location.x(),location.y(),0.0f);    // Move to centre of goal.
    glRotatef(angle,0.0f,0.0f,1.0f);				// Rotate The ellipse On It's Z Axis

    GLfloat glLineColour[4];
    qSetColor(glLineColour, lineColour);

    GLfloat glFillColour[4];
    qSetColor(glFillColour, fillColour);

    glColor4fv(glFillColour);

    glBegin(GL_POLYGON);
    for (int i=0; i < 360; i++)
    {
        //convert degrees into radians
        float degInRad = i*DEG2RAD;
        glVertex2f(cos(degInRad)*xradius,sin(degInRad)*yradius);
    }
    glEnd();

    glColor4fv(glLineColour);
    glBegin(GL_LINE_LOOP);
    for (int i=0; i < 360; i++)
    {
        //convert degrees into radians
        float degInRad = i*DEG2RAD;
        glVertex2f(cos(degInRad)*xradius,sin(degInRad)*yradius);
    }
    glEnd();

    glPopMatrix();
    glDisable(GL_BLEND);		// Turn Blending On
    glEnable(GL_DEPTH_TEST);		// Turn Z Buffer testing On
    glEnable(GL_LIGHTING);      // Enable Global Lighting
}

void locWmGlDisplay::DrawLocalisationOverlay(const Localisation& localisation, const QColor& modelColor)
{
    if(!drawBestModelOnly)
    {
        QString displayString("Model %1 (%2%)");
        for(int modelID = 0; modelID < Localisation::c_MAX_MODELS; modelID++)
        {
            const KF model = localisation.getModel(modelID);
            if(model.active())
            {
                glDisable(GL_LIGHTING);      // Enable Global Lighting
                glDisable(GL_DEPTH_TEST);		// Turn Z Buffer testing Off
                glColor4ub(255,255,255,255);
                renderText(model.state(KF::selfX), model.state(KF::selfY),1,displayString.arg(modelID).arg(model.alpha()*100,0,'f',1));
                glEnable(GL_DEPTH_TEST);		// Turn Z Buffer testing On
                glEnable(GL_LIGHTING);      // Enable Global Lighting
            }
        }
    }
}

void locWmGlDisplay::drawStationaryObjectLabel(const StationaryObject& object)
{
    QString displayString("(%1,%2)");
    renderText(object.X(), object.Y(),1,displayString.arg(object.measuredDistance(),0,'f',1).arg(object.measuredBearing(),0,'f',3));
}

void locWmGlDisplay::drawFieldObjectLabels(const FieldObjects& theFieldObjects)
{
    glDisable(GL_DEPTH_TEST);		// Turn Z Buffer testing Off
    for (unsigned int i = 0; i < theFieldObjects.stationaryFieldObjects.size(); i++)
    {
        if(theFieldObjects.stationaryFieldObjects[i].isObjectVisible())
            drawStationaryObjectLabel(theFieldObjects.stationaryFieldObjects[i]);
    }
    glEnable(GL_DEPTH_TEST);		// Turn Z Buffer testing On
}

void locWmGlDisplay::drawLegend(QPainter* painter)
{
    const QSize blockSize(16,16);
    const QPoint legendOrigin(20, 20);

    std::vector<QColor> colours;
    colours.push_back(m_currentColour);
    colours.push_back(m_localColour);
    colours.push_back(m_selfColour);
    colours.push_back(m_fieldObjColour);
    colours.push_back(m_sensorColour);

    std::vector<QString> legendLabel;
    legendLabel.push_back("Localisation log");
    legendLabel.push_back("Original localisation");
    legendLabel.push_back("New localisation");
    legendLabel.push_back("Field Object Self");
    legendLabel.push_back("Measured Position");


    std::vector<float> temp(3,0);
    std::vector<bool> valid;
    valid.push_back(currentLocalisation != NULL);
    valid.push_back(localLocalisation != NULL);
    valid.push_back(m_self_loc != NULL);
    valid.push_back(currentObjects != NULL);
    valid.push_back((currentSensorData != NULL) and currentSensorData->getGps(temp));

    unsigned int currLegendKey = 0;
    for(unsigned int i = 0; i < colours.size(); ++i)
    {
        if(not valid.at(i)) continue;
        QPixmap colourBlock(blockSize);
        colourBlock.fill();
        QPainter pixPaint(&colourBlock);
        pixPaint.fillRect(1,1,14,14,colours.at(i));
        pixPaint.end();
        QPoint blockDrawPos = legendOrigin + QPoint(0, currLegendKey*20);
        QPoint textDrawPos = blockDrawPos + QPoint(20, 12);
        painter->drawPixmap(blockDrawPos, colourBlock);

        QPen pen;
        pen.setColor(Qt::white);
        painter->setPen(pen);
        painter->drawText(textDrawPos, legendLabel.at(i));
        currLegendKey++;

//        QPainterPath path;
//        QFont font = painter->font();
//        pen.setWidth(1);

//        painter->setBrush(QBrush(Qt::black));

//        path.addText(textDrawPos, font, legendLabel.at(i)); //Adjust the position
//        painter->drawPath(path);
    }
}

void locWmGlDisplay::resizeGL(int width, int height)
{
    setupViewport(width, height);
//        glViewport(0, 0, width, height);
//        glMatrixMode(GL_PROJECTION);						// Select The Projection Matrix
//        glLoadIdentity();							// Reset The Projection Matrix
//        if(perspective)
//        {
//            gluPerspective(45.0f,(GLfloat)width/(GLfloat)height,0.1f,10000.0f);
//        }
//        else
//        {
//            glOrtho(-370.0,  370.0, 270.0, -270.0,0.1,10000.0);
//        }
//        // Calculate The Aspect Ratio Of The Window


//        glMatrixMode(GL_MODELVIEW);						// Select The Modelview Matrix
//        glLoadIdentity();							// Reset The Modelview Matrix
    return;
}

QSize locWmGlDisplay::minimumSizeHint() const
{
        return QSize(50, 50);
}

QSize locWmGlDisplay::sizeHint() const
{
        return QSize(320, 240);
}
