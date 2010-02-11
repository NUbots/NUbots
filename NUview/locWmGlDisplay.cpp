#include "locWmGlDisplay.h"
#include <QDebug>
#include <QMouseEvent>


locWmGlDisplay::locWmGlDisplay(QWidget *parent): QGLWidget(parent)
{
    setWindowTitle("World Model Visualisation");
    viewTranslation[0] = 0.0f;
    viewTranslation[1] = 0.0f;
    viewTranslation[2] = -700.0f;
    viewOrientation[0] = 0.0f;
    viewOrientation[1] = 0.0f;
    viewOrientation[2] = 0.0f;
}

locWmGlDisplay::~locWmGlDisplay()
{
    return;
}

void locWmGlDisplay::mousePressEvent ( QMouseEvent * event )
{
    if(event->button()==Qt::RightButton)
    {
        dragStartPosition = event->pos();
        prevDragPos = dragStartPosition;
        qDebug() << "Drag Started!";
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
}

void locWmGlDisplay::initializeGL()
{
    glShadeModel(GL_SMOOTH);						// Enables Smooth Shading
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);					// Black Background

    glClearDepth(1.0f);						// Depth Buffer Setup
    glEnable(GL_DEPTH_TEST);						// Enables Depth Testing
    glDepthFunc(GL_LEQUAL);						// The Type Of Depth Test To Do

    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);			// Really Nice Perspective Calculations
    glEnable (GL_LINE_SMOOTH);

    glEnable(GL_TEXTURE_2D);                                            // Enable Texture Mapping

    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);                   // Setup Alpha Blending
    QPixmap textureImage(QString(":/textures/FieldLines.png"));
    QPixmap grassTextureImage(QString(":/textures/grass_texture.jpg"));
    texture = bindTexture(textureImage);
    grassTexture = bindTexture(grassTextureImage);

    quadratic = gluNewQuadric();
    gluQuadricNormals(quadratic, GLU_SMOOTH);	// Create Smooth Normals ( NEW )
    gluQuadricTexture(quadratic, GL_TRUE);		// Create Texture Coords ( NEW )


    // Lighting


    GLfloat LightPosition[]= { 0.0f, 0.0f, 0.0f, 1.0f };				 // Light Position ( NEW )
    GLfloat LightAmbient[]= { 0.0f, 0.0f, 0.0f, 1.0f }; // Ambient Light Values ( NEW )
    GLfloat LightDiffuse[]= { 1.0f, 1.0f, 1.0f, 1.0f };				 // Diffuse Light Values ( NEW )
    GLfloat LightSpecular[]= { 1.0f, 1.0f, 1.0f, 1.0f };				 // Diffuse Light Values ( NEW )
    GLfloat MatSpecular[]= { 0.0f, 0.0f, 0.0f, 1.0f };				 // Diffuse Light Values ( NEW )
    GLfloat MatEmission[]= { 0.0f, 0.0f, 0.0f, 1.0f };				 // Diffuse Light Values ( NEW )

    glLightfv(GL_LIGHT1, GL_POSITION,LightPosition);			// Position The Light
    glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbient);				// Setup The Ambient Light
    glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse);				// Setup The Diffuse Light
    glLightfv(GL_LIGHT1, GL_SPECULAR, LightSpecular);				// Setup The Diffuse Light
    glEnable(GL_LIGHT1);							// Enable Light One


    glEnable(GL_LIGHTING);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT, GL_DIFFUSE);
    glMaterialfv(GL_FRONT,GL_SPECULAR,MatSpecular);
    glMaterialfv(GL_FRONT,GL_EMISSION,MatEmission);
    glMaterialf(GL_FRONT_AND_BACK,GL_SHININESS,128.0f);
    return;
}

void locWmGlDisplay::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);			// Clear The Screen And The Depth Buffer
    glLoadIdentity();							// Reset The Current Modelview Matrix

    glTranslatef(viewTranslation[0],viewTranslation[1],viewTranslation[2]);					// Move Left 1.5 Units And Into The Screen 6.0
    glRotatef(viewOrientation[0],1.0f,0.0f,0.0f); // Rotate about X-Axis
    glRotatef(viewOrientation[1],0.0f,1.0f,0.0f); // Rotate about Y-Axis
    glRotatef(viewOrientation[2],0.0f,0.0f,1.0f); // Rotate about Z-Axis
    drawField();
    drawBall(QColor(255,165,0,255), 0.0f, 0.0f);

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
            glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST);    // Turn off filtering of textures
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);    // Turn off filtering of textures
    glBegin(GL_QUADS);
        glNormal3f(0.0f,0.0f,1.0f);	// Set The Normal
        glTexCoord2f(0.0f, 0.0f); glVertex3f(-370.0f,  270.0f,  0.0f);      // Bottom Left Of The Texture and Quad
        glTexCoord2f(7.4f, 0.0f); glVertex3f( 370.0f,  270.0f,  0.0f);    // Bottom Right Of The Texture and Quad
        glTexCoord2f(7.4f, 5.4f); glVertex3f( 370.0f, -270.0f,  0.0f);     // Top Right Of The Texture and Quad
        glTexCoord2f(0.0f, 5.4f); glVertex3f(-370.0f, -270.0f,  0.0f);       // Top Left Of The Texture and Quad
    glEnd();

    glBindTexture(GL_TEXTURE_2D, texture);
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);    // Turn off filtering of textures
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);    // Turn off filtering of textures
    glBegin(GL_QUADS);
        glNormal3f(0.0f,0.0f,1.0f);	// Set The Normal
        glTexCoord2f(0.0f, 0.0f); glVertex3f(-370.0f,  270.0f,  0.0f);      // Bottom Left Of The Texture and Quad
        glTexCoord2f(1.0f, 0.0f); glVertex3f( 370.0f,  270.0f,  0.0f);    // Bottom Right Of The Texture and Quad
        glTexCoord2f(1.0f, 1.0f); glVertex3f( 370.0f, -270.0f,  0.0f);     // Top Right Of The Texture and Quad
        glTexCoord2f(0.0f, 1.0f); glVertex3f(-370.0f, -270.0f,  0.0f);       // Top Left Of The Texture and Quad
    glEnd();

    glDisable(GL_TEXTURE_2D);                                            // Enable Texture Mapping
    glEnable(GL_BLEND);		// Turn Blending On
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    drawGoal(QColor(0,0,255,255),-300,0.0,0.0);
    drawGoal(QColor(255,255,0,255),300,0.0,180.0);
    
    glPopMatrix();
}

void locWmGlDisplay::drawGoal(QColor colour, float x, float y, float facing)
{
    float postRadius = 5;
    float crossBarRadius = 2.5;
    glPushMatrix();
    glColor4f(colour.red(),colour.green(),colour.blue(),colour.alpha());
    glDisable(GL_TEXTURE_2D);       // Disable Texture Mapping
    glTranslatef(x,y,0.0f);    // Move to centre of goal.
    glRotatef(facing,0.0f,0.0f,1.0f);				// Rotate The Pyramid On It's Y Axis
    glTranslatef(0.0f,70.0f,0.0f);    // Move to right post.
    gluCylinder(quadratic,postRadius,postRadius,80.0f,32,32);	// Draw right post

    glColor4f(1.0f,1.0f,1.0f,colour.alpha());

        glBegin(GL_TRIANGLES);						// Drawing Using Triangles
                glVertex3f( -postRadius, 0.0f, 40.0f);				// Top
                glVertex3f(-postRadius - 40.0f,0.0f, 0.0f);				// Bottom Left
                glVertex3f( -postRadius,0.0f, 0.0f);				// Bottom Right
        glEnd();							// Finished Drawing The Triangle


    glTranslatef(0.0f,-2*70.0f,0.0f);    // Move to left post.
    glColor4f(colour.red(),colour.green(),colour.blue(),colour.alpha());
    gluCylinder(quadratic,postRadius,postRadius,80.0f,32,32);	// Draw left post
    glColor4f(1.0f,1.0f,1.0f,colour.alpha());

        glBegin(GL_TRIANGLES);						// Drawing Using Triangles
                glVertex3f( -postRadius, 0.0f, 40.0f);				// Top
                glVertex3f(-postRadius - 40.0f,0.0f, 0.0f);				// Bottom Left
                glVertex3f( -postRadius,0.0f, 0.0f);				// Bottom Right
        glEnd();							// Finished Drawing The Triangle

    glTranslatef(0.0f,-postRadius,80.0f - crossBarRadius);    // Move to top of left post.
    glRotatef(-90.0,1.0f,0.0f,0.0f);
    glColor4f(colour.red(),colour.green(),colour.blue(),colour.alpha());
    gluCylinder(quadratic,crossBarRadius,crossBarRadius,140.0f+2.0f*postRadius,32,32);	// Draw cross bar
    glPopMatrix();

}

void locWmGlDisplay::drawBall(QColor colour, float x, float y)
{
    float ballRadius = 6.5/2.0;

    glPushMatrix();
    glColor4f(colour.red(),colour.green(),colour.blue(),colour.alpha());
    glTranslatef(x,y,ballRadius);    // Move to centre of goal.
    gluSphere(quadratic,ballRadius,32,32);		// Draw A Sphere
    glPopMatrix();
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
        return QSize(160, 120);
}
