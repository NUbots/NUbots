#include "locWmGlDisplay.h"
#include <QDebug>
#include <QMouseEvent>
#include "Tools/Math/General.h"
#include "Localisation/Localisation.h"


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
}

locWmGlDisplay::~locWmGlDisplay()
{
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
    if(currentLocalisation)
    {
        DrawLocalisation(*currentLocalisation);
    }
    //drawBall(QColor(255,165,0,255), 0.0f, 0.0f);    // Draw the ball.
    //drawRobot(QColor(255,255,255,255), 30.0f, 30.0f, 0.75f);
    glFlush ();         // Run Queued Commands
    return;
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
    glColor4ub(colour.red(),colour.green(),colour.blue(),colour.alpha());
    glTranslatef(x,y,ballRadius);    // Move to centre of ball.
    gluSphere(quadratic,ballRadius,128,128);		// Draw A Sphere
    glColor4f(1.0f,1.0f,1.0f,1.0f); // Back to white
    glPopMatrix();
}

void locWmGlDisplay::drawRobot(QColor colour, float x, float y, float theta)
{
    const float robotHeight = 58.0f;
    const float robotWidth = 30.0f;
    glPushMatrix();
    glEnable(GL_BLEND);		// Turn Blending On
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_TEXTURE_2D);       // Disable Texture Mapping
    glTranslatef(x,y,0);    // Move to centre of robot.
    glRotatef(mathGeneral::rad2deg(theta),0.0f, 0.0f, 1.0f);

    // Draw Aura
    glColor4ub(0,0,255,255);
    glBindTexture(GL_TEXTURE_2D, robotAuraTexture);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);    // Turn off filtering of textures
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);    // Turn off filtering of textures
    glBegin(GL_QUADS);
        glNormal3f(0.0f,0.0f,1.0f);	// Set The Normal
        glTexCoord2f(0.0f, 0.0f); glVertex3f(-robotWidth/2.0f,  -robotWidth/2.0f*1.2,  0.5);      // Bottom Left Of The Texture and Quad
        glTexCoord2f(1.0f, 0.0f); glVertex3f(-robotWidth/2.0f, robotWidth/2.0f*1.2,  0.5);    // Bottom Right Of The Texture and Quad
        glTexCoord2f(1.0f, 1.0f); glVertex3f(+robotWidth/2.0f, robotWidth/2.0f*1.2,  0.5);     // Top Right Of The Texture and Quad
        glTexCoord2f(0.0f, 1.0f); glVertex3f(+robotWidth/2.0f, -robotWidth/2.0f*1.2,  0.5);       // Top Left Of The Texture and Quad
    glEnd();

    glColor4ub(255,255,255,colour.alpha());
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

    glDisable(GL_TEXTURE_2D);       // Disable Texture Mapping
    glDisable(GL_BLEND);		// Turn Blending On
    glPopMatrix();
}

void locWmGlDisplay::DrawModel(const KF& model)
{
    drawRobot(QColor(255,255,255,model.alpha*255), model.getState(KF::selfX), model.getState(KF::selfY), model.getState(KF::selfTheta));
    drawBall(QColor(255,165,0,255), model.getState(KF::ballX), model.getState(KF::ballY));
}

void locWmGlDisplay::DrawLocalisation(const Localisation& localisation)
{
    for(int modelID = 0; modelID < Localisation::c_MAX_MODELS; modelID++)
    {
        const KF model = localisation.getModel(modelID);
        if(model.isActive) DrawModel(model);
    }

}

void locWmGlDisplay::resizeGL(int width, int height)
{
        glViewport(0, 0, width, height);
        glMatrixMode(GL_PROJECTION);						// Select The Projection Matrix
        glLoadIdentity();							// Reset The Projection Matrix

        // Calculate The Aspect Ratio Of The Window
        gluPerspective(45.0f,(GLfloat)width/(GLfloat)height,0.1f,10000.0f);

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
