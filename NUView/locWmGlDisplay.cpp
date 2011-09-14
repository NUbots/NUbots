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

locWmGlDisplay::locWmGlDisplay(QWidget *parent): QGLWidget(parent), currentLocalisation(0)
{
    setWindowTitle("World Model Visualisation");
    viewTranslation[0] = 0.0f;
    viewTranslation[1] = 0.0f;
    viewTranslation[2] = -700.0f;
    viewOrientation[0] = 0.0f;
    viewOrientation[1] = 0.0f;
    viewOrientation[2] = 0.0f;
    setFocusPolicy(Qt::StrongFocus); // Required to get keyboard events.
    light = true;
    perspective = true;
    drawRobotModel = false;
    drawSigmaPoints = false;
    drawBestModelOnly = false;
    setFont(QFont("Helvetica",12,QFont::Bold,false));
    currentLocalisation = 0;
    currentObjects = 0;
    localLocalisation = 0;
    currentSensorData = 0;
    m_self_loc = 0;
}

locWmGlDisplay::~locWmGlDisplay()
{
    makeCurrent();
    return;
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
            viewOrientation[0] = 0.0f;
            viewOrientation[1] = 0.0f;
            viewOrientation[2] = 0.0f;
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
        case Qt::Key_B:
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
    switch(event->button())
    {
        case Qt::RightButton:
            dragStartPosition = event->pos();
            prevDragPos = dragStartPosition;
            break;
        default:
            QGLWidget::mousePressEvent(event);
    }
}

void locWmGlDisplay::mouseMoveEvent ( QMouseEvent * event )
{
    if((event->buttons()&Qt::RightButton) && (event->buttons()&Qt::LeftButton)){
        QPoint currPos = event->pos();
        viewTranslation[2] += (currPos.y() - prevDragPos.y());
        prevDragPos = currPos;
        update();
    }
    else if(event->buttons()&Qt::RightButton){
        QPoint currPos = event->pos();
        viewOrientation[0] += (currPos.y() - prevDragPos.y());
        viewOrientation[2] += (currPos.x() - prevDragPos.x());
        prevDragPos = currPos;
        update();
    }
    else
    {
        QGLWidget::mouseMoveEvent(event);
    }
}

void locWmGlDisplay::wheelEvent ( QWheelEvent * event )
{
    int magnitude = event->delta();
    float zoomDistance = (float)magnitude / 10.0f;
    viewTranslation[2] += zoomDistance;
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
    GLfloat LightAmbient[]= { 0.0f, 0.0f, 0.0f, 1.0f };         // Ambient Light Values
    GLfloat LightDiffuse[]= { 1.0f, 1.0f, 1.0f, 1.0f };		// Diffuse Light Values
    GLfloat LightSpecular[]= { 1.0f, 1.0f, 1.0f, 1.0f };        // Diffuse Light Values
    GLfloat LightPosition[]= { 0.0f, 0.0f, 200.0f, 1.0f };	// Light Position

    quadratic = gluNewQuadric();
    gluQuadricNormals(quadratic, GLU_SMOOTH);	// Create Smooth Normals

    glEnable(GL_TEXTURE_2D);                    // Enable Texture Mapping
    glShadeModel(GL_SMOOTH);    		// Enable Smooth Shading

    glClearColor (0.0, 0.0, 0.0, 0.0);
    glClearDepth(1.0f);                         		// Depth Buffer Setup
    glEnable(GL_DEPTH_TEST);					// Enables Depth Testing
    glDepthFunc(GL_LEQUAL);					// The Type Of Depth Test To Do
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);		// Really Nice Perspective Calculations


    loadTexture(":/textures/robotAura.png", &robotAuraTexture);
    loadTexture(":/textures/FieldLines.png", &fieldLineTexture);
    loadTexture(":/textures/grass_texture.jpg", &grassTexture);
    loadTexture(":/textures/PlaceholderRobot.png", &robotTexture);
    loadTexture(":/textures/PlaceholderRobotBack.png", &robotBackTexture);

    // Lighting
    glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbient);			// Setup The Ambient Light
    glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse);			// Setup The Diffuse Light
    glLightfv(GL_LIGHT1, GL_SPECULAR,LightSpecular);			// Setup Specular Light
    glLightfv(GL_LIGHT1, GL_POSITION,LightPosition);			// Position The Light

    glEnable(GL_LIGHT1);						// Enable Light One

    glEnable(GL_LIGHTING);      // Enable Global Lighting
}

void locWmGlDisplay::paintGL()
{
    makeCurrent();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);         // Clear The Screen And The Depth Buffer
    glLoadIdentity();						// Reset The Current Modelview Matrix

    glEnable(GL_COLOR_MATERIAL);                                // Enable Material Colour Tracking
    GLfloat MatSpecular[]= { 1.0f, 1.0f, 1.0f, 1.0f };          // Diffuse Material Values
    GLfloat MatEmission[]= { 0.0f, 0.0f, 0.0f, 1.0f };  	// Diffuse Material Values

    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE); // Set Ambient and Diffuse Material Values to Track
    glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,MatSpecular);
    glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,MatEmission);
    glMaterialf(GL_FRONT_AND_BACK,GL_SHININESS,100.0f);

    glTranslatef(viewTranslation[0],viewTranslation[1],viewTranslation[2]); // Move to centre of field position.
    glRotatef(viewOrientation[0],1.0f,0.0f,0.0f); // Rotate about X-Axis
    glRotatef(viewOrientation[1],0.0f,1.0f,0.0f); // Rotate about Y-Axis
    glRotatef(viewOrientation[2],0.0f,0.0f,1.0f); // Rotate about Z-Axis

    // Position Lighting
    GLfloat LightPosition[]= { 0.0f, 0.0f, 300.0f, 1.0f };      // Light Position
    glLightfv(GL_LIGHT1, GL_POSITION,LightPosition);		// Set Light Position

    drawField();        // Draw the Standard Field Layout.

    drawMarkers();
    drawObjects();
    drawOverlays();

    glFlush ();         // Run Queued Commands
    return;
}

void locWmGlDisplay::drawMarkers()
{
    if(currentSensorData)
    {
        QColor sensorColor(0,0,0);
        std::vector<float> gpsPosition;
        float compassHeading;
        if(currentSensorData->getGps(gpsPosition))
        {
            if(!currentSensorData->getCompass(compassHeading)) compassHeading = 3.14;
            drawRobotMarker(sensorColor, gpsPosition[0], gpsPosition[1],compassHeading);
        }

    }
    if(currentObjects)
    {
        QColor currentColor(255,0,0);
        drawRobotMarker(currentColor, currentObjects->self.wmX(), currentObjects->self.wmY(),currentObjects->self.Heading());
    }

    if(currentLocalisation)
    {
        QColor currentColor(0,0,255);
        DrawLocalisationMarkers(*currentLocalisation, currentColor);
    }
    if(localLocalisation)
    {
        QColor localColor(255,255,0);
        DrawLocalisationMarkers(*localLocalisation, localColor);
    }
    if(m_self_loc)
    {
        QColor self_color(142, 56, 142);
        drawLocalisationMarkers(*m_self_loc, self_color);
    }
}

void locWmGlDisplay::drawObjects()
{
    if(currentSensorData)
    {
        QColor sensorColor(0,0,0);
        std::vector<float> gpsPosition;
        float compassHeading;
        if(currentSensorData->getGps(gpsPosition))
        {
            if(!currentSensorData->getCompass(compassHeading)) compassHeading = 3.14;
            drawRobot(sensorColor, gpsPosition[0], gpsPosition[1],compassHeading);
        }

    }
    if(currentLocalisation)
    {
        QColor currentColor(0,0,255);
        DrawLocalisationObjects(*currentLocalisation, currentColor);
    }
    if(localLocalisation)
    {
        QColor localColor(255,255,0);
        DrawLocalisationObjects(*localLocalisation, localColor);
    }
}

void locWmGlDisplay::drawOverlays()
{
    if(currentLocalisation)
    {
        QColor currentColor(0,0,255);
        DrawLocalisationOverlay(*currentLocalisation, currentColor);
    }
    if(currentObjects)
    {
        drawFieldObjectLabels(*currentObjects);
    }
    if(localLocalisation)
    {
        QColor localColor(255,255,0);
        DrawLocalisationOverlay(*localLocalisation, localColor);
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
    glEnable(GL_BLEND);		// Turn Blending On
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

    glColor3f(1.0f,1.0f,1.0f);                                // Set The Color To White

    glEnable(GL_TEXTURE_2D);                                            // Enable Texture Mapping
    glBindTexture(GL_TEXTURE_2D, grassTexture);
            glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);    // Turn off filtering of textures
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);    // Turn off filtering of textures
    glBegin(GL_QUADS);
        glNormal3f(0.0f,0.0f,1.0f);	// Set The Normal
        glTexCoord2f(0.0f, 0.0f); glVertex3f(-370.0f,  270.0f,  0.0f);      // Bottom Left Of The Texture and Quad
        glTexCoord2f(7.4f, 0.0f); glVertex3f( 370.0f,  270.0f,  0.0f);    // Bottom Right Of The Texture and Quad
        glTexCoord2f(7.4f, 5.4f); glVertex3f( 370.0f, -270.0f,  0.0f);     // Top Right Of The Texture and Quad
        glTexCoord2f(0.0f, 5.4f); glVertex3f(-370.0f, -270.0f,  0.0f);       // Top Left Of The Texture and Quad
    glEnd();

    glBindTexture(GL_TEXTURE_2D, fieldLineTexture);
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);    // Turn off filtering of textures
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);    // Turn off filtering of textures
    glBegin(GL_QUADS);
        glNormal3f(0.0f,0.0f,1.0f);	// Set The Normal
        glTexCoord2f(0.0f, 0.0f); glVertex3f(-370.0f,  270.0f,  0.0f);      // Bottom Left Of The Texture and Quad
        glTexCoord2f(1.0f, 0.0f); glVertex3f( 370.0f,  270.0f,  0.0f);    // Bottom Right Of The Texture and Quad
        glTexCoord2f(1.0f, 1.0f); glVertex3f( 370.0f, -270.0f,  0.0f);     // Top Right Of The Texture and Quad
        glTexCoord2f(0.0f, 1.0f); glVertex3f(-370.0f, -270.0f,  0.0f);       // Top Left Of The Texture and Quad
    glEnd();
    glDisable(GL_BLEND);            // Turn Blending Off
    glDisable(GL_TEXTURE_2D);       // Disable Texture Mapping

    drawGoal(QColor(0,0,255,255),-300,0.0,0.0);
    drawGoal(QColor(255,255,0,255),300,0.0,180.0);
    glPopMatrix();
}

void locWmGlDisplay::drawGoal(QColor colour, float x, float y, float facing)
{
    float postRadius = 5;
    float crossBarRadius = 2.5;
    glPushMatrix();
    glColor4ub(colour.red(),colour.green(),colour.blue(),colour.alpha());

    glTranslatef(x,y,0.0f);    // Move to centre of goal.
    glRotatef(facing,0.0f,0.0f,1.0f);				// Rotate The Pyramid On It's Y Axis
    glTranslatef(0.0f,70.0f,0.0f);    // Move to right post.
    gluCylinder(quadratic,postRadius,postRadius,80.0f,128,128);	// Draw right post

    glColor4f(1.0f,1.0f,1.0f,colour.alpha());

    glBegin(GL_TRIANGLES);				// Drawing Using Triangles
        glVertex3f( -postRadius, 0.0f, 40.0f);		// Top
        glVertex3f(-postRadius - 40.0f,0.0f, 0.0f);	// Bottom Left
        glVertex3f( -postRadius,0.0f, 0.0f);		// Bottom Right
    glEnd();						// Finished Drawing The Triangle


    glTranslatef(0.0f,-2*70.0f,0.0f);    // Move to left post.
    glColor4ub(colour.red(),colour.green(),colour.blue(),colour.alpha());
    gluCylinder(quadratic,postRadius,postRadius,80.0f,128,128);	// Draw left post
    glColor4f(1.0f,1.0f,1.0f,colour.alpha());

    glBegin(GL_TRIANGLES);				// Drawing Using Triangles
        glVertex3f( -postRadius, 0.0f, 40.0f);		// Top
        glVertex3f(-postRadius - 40.0f,0.0f, 0.0f);	// Bottom Left
        glVertex3f( -postRadius,0.0f, 0.0f);		// Bottom Right
    glEnd();						// Finished Drawing The Triangle

    glTranslatef(0.0f,-postRadius,80.0f - crossBarRadius);    // Move to top of left post.
    glRotatef(-90.0,1.0f,0.0f,0.0f);
    glColor4ub(colour.red(),colour.green(),colour.blue(),colour.alpha());
    gluCylinder(quadratic,crossBarRadius,crossBarRadius,140.0f+2.0f*postRadius,128,128);	// Draw cross bar
    glColor4f(1.0f,1.0f,1.0f,1.0f); // Back to white
    glPopMatrix();

}

void locWmGlDisplay::drawBall(QColor colour, float x, float y)
{
    const float ballRadius = 6.5/2.0;

    glPushMatrix();
    glEnable(GL_BLEND);		// Turn Blending On
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    glColor4ub(colour.red(),colour.green(),colour.blue(),colour.alpha());
    glTranslatef(x,y,ballRadius);    // Move to centre of ball.
    gluSphere(quadratic,ballRadius,128,128);		// Draw A Sphere
    glColor4f(1.0f,1.0f,1.0f,1.0f); // Back to white
    glDisable(GL_BLEND);		// Turn Blending On
    glPopMatrix();
}

void locWmGlDisplay::drawBallMarker(QColor colour, float x, float y)
{
    const float ballRadius = 6.5/2.0;

    glPushMatrix();
    glEnable(GL_BLEND);		// Turn Blending On
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_DEPTH_TEST);		// Turn Z Buffer testing Off
    glDisable(GL_LIGHTING);      // Disable Global Lighting
    glColor4ub(colour.red(), colour.green(), colour.blue(), colour.alpha());
    glTranslatef(x,y,0);    // Move to centre of ball.
    gluDisk(quadratic, ballRadius, ballRadius+2, 32, 32);
    glColor4f(1.0f,1.0f,1.0f,1.0f); // Back to white
    glEnable(GL_DEPTH_TEST);		// Turn Z Buffer testing On
    glEnable(GL_LIGHTING);      // Enable Global Lighting
    glDisable(GL_BLEND);		// Turn Blending On
    glPopMatrix();
    return;
}

void locWmGlDisplay::drawRobotMarker(QColor colour, float x, float y, float theta)
{
    const float robotWidth = 30.0f;
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

    glColor4ub(255,255,255,colour.alpha());
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

    glPushMatrix();
    glEnable(GL_BLEND);		// Turn Blending On
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_DEPTH_TEST);		// Turn Z Buffer testing Off
    glDisable(GL_LIGHTING);      // Disable Global Lighting
    glColor4ub(colour.red(), colour.green(), colour.blue(), colour.alpha());
    glTranslatef(x,y,0);    // Move to centre of ball.
    gluDisk(quadratic, ballRadius, ballRadius+2, 32, 32);
    glColor4f(1.0f,1.0f,1.0f,1.0f); // Back to white
    glEnable(GL_DEPTH_TEST);		// Turn Z Buffer testing On
    glEnable(GL_LIGHTING);      // Enable Global Lighting
    glDisable(GL_BLEND);		// Turn Blending On
    glPopMatrix();
    return;
}

void locWmGlDisplay::DrawModelObjects(const KF& model, QColor& modelColor)
{
    int alpha = 255*model.alpha();
    if(alpha < 25) alpha = 25;
    modelColor.setAlpha(alpha);
    drawRobot(modelColor, model.state(KF::selfX), model.state(KF::selfY), model.state(KF::selfTheta));
    if(drawSigmaPoints)
    {
        Matrix sigmaPoints = model.CalculateSigmaPoints();
        for (int i=1; i < sigmaPoints.getn(); i++)
        {
            DrawSigmaPoint(modelColor, sigmaPoints[KF::selfX][i], sigmaPoints[KF::selfY][i], sigmaPoints[KF::selfTheta][i]);
        }
    }
    drawBall(QColor(255,165,0,alpha), model.state(KF::ballX), model.state(KF::ballY));
}

void locWmGlDisplay::DrawLocalisationObjects(const Localisation& localisation, QColor& modelColor)
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
                DrawModelObjects(model, modelColor);
            }
        }
    }
}

void locWmGlDisplay::DrawModelMarkers(const KF& model, QColor& modelColor)
{

    drawRobotMarker(modelColor, model.state(KF::selfX), model.state(KF::selfY), model.state(KF::selfTheta));
    if(drawSigmaPoints)
    {
        Matrix sigmaPoints = model.CalculateSigmaPoints();
        for (int i=1; i < sigmaPoints.getn(); i++)
        {
            DrawBallSigma(modelColor, sigmaPoints[KF::ballX][i], sigmaPoints[KF::ballY][i]);
            DrawSigmaPoint(modelColor, sigmaPoints[KF::selfX][i], sigmaPoints[KF::selfY][i], sigmaPoints[KF::selfTheta][i]);
        }
    }
    drawBallMarker(modelColor, model.state(KF::ballX), model.state(KF::ballY));
}

void locWmGlDisplay::DrawModelMarkers(const Model* model, QColor& modelColor)
{
    drawRobotMarker(modelColor, model->mean(Model::states_x), model->mean(Model::states_y), model->mean(Model::states_heading));
    if(drawSigmaPoints)
    {
        Matrix sigmaPoints = model->CalculateSigmaPoints();
        for (int i=1; i < sigmaPoints.getn(); i++)
        {
            DrawSigmaPoint(modelColor, sigmaPoints[Model::states_x][i], sigmaPoints[Model::states_y][i], sigmaPoints[Model::states_heading][i]);
        }
    }
}

void locWmGlDisplay::DrawLocalisationMarkers(const Localisation& localisation, QColor& modelColor)
{
    const int c_min_display_alpha = 50; // Minimum alpha to use when drawing a model.
    if(drawBestModelOnly)
    {
        modelColor.setAlpha(255);
        const KF model = localisation.getBestModel();
        DrawModelMarkers(model, modelColor);
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
                modelColor.setAlpha(alpha);
                DrawModelMarkers(model, modelColor);
            }
        }
    }
}

void locWmGlDisplay::drawLocalisationMarkers(const SelfLocalisation& localisation, QColor& modelColor)
{
    const int c_min_display_alpha = 50; // Minimum alpha to use when drawing a model.
    if(drawBestModelOnly)
    {
        modelColor.setAlpha(255);
        const Model* model = localisation.getBestModel();
        DrawModelMarkers(model, modelColor);
    }
    else
    {
        QString displayString("Model %1 (%2%)");
        ModelContainer models = localisation.allModels();
        for(ModelContainer::const_iterator model_it = models.begin(); model_it != models.end(); ++model_it)
        {
            if((*model_it)->active())
            {
                int alpha = std::max(c_min_display_alpha, (int)(255*(*model_it)->alpha()));
                modelColor.setAlpha(alpha);
                DrawModelMarkers((*model_it), modelColor);
            }
        }
    }
}

void locWmGlDisplay::DrawLocalisationOverlay(const Localisation& localisation, QColor& modelColor)
{
    QColor draw_colour = modelColor;
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

void locWmGlDisplay::resizeGL(int width, int height)
{
        glViewport(0, 0, width, height);
        glMatrixMode(GL_PROJECTION);						// Select The Projection Matrix
        glLoadIdentity();							// Reset The Projection Matrix
        if(perspective)
        {
            gluPerspective(45.0f,(GLfloat)width/(GLfloat)height,0.1f,10000.0f);
        }
        else
        {
            glOrtho(-370.0,  370.0, 270.0, -270.0,0.1,10000.0);
        }
        // Calculate The Aspect Ratio Of The Window


        glMatrixMode(GL_MODELVIEW);						// Select The Modelview Matrix
        glLoadIdentity();							// Reset The Modelview Matrix
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
