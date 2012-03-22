#include "ColorModelConversions.h"
#include "LUTGlDisplay.h"
#include <QDebug>
#include <QMouseEvent>
#include <qclipboard.h>
#include <QApplication>
#include <glu.h>
#include "Tools/Math/General.h"
#include "Tools/FileFormats/LUTTools.h"


/*!
LUTViewer: to view loaded LUTs. Changes when LUTs are updated in classification phase, or when they are loaded. A widget for NUview QT application.
@author Aaron Wong

*/

LUTGlDisplay::LUTGlDisplay(QWidget *parent, const OpenglManager * shareWidget): QGLWidget(parent,(QGLWidget*)shareWidget), currentLUT(0)
{
    setWindowTitle("Look Up Table (LUT) Visualisation");
    viewTranslation[0] = -64.0f;
    viewTranslation[1] = -64.0f;
    viewTranslation[2] = -300.0f;
    viewOrientation[0] = 0.0f;
    viewOrientation[1] = 0.0f;
    viewOrientation[2] = 0.0f;
    setFocusPolicy(Qt::StrongFocus); // Required to get keyboard events.
    light = true;
    perspective = true;
    trueColours = false;
    setFont(QFont("Helvetica",12,QFont::Bold,false));
}

LUTGlDisplay::~LUTGlDisplay()
{
    return;
}

void LUTGlDisplay::restoreState(const QByteArray & state)
{
    return;
}

QByteArray LUTGlDisplay::saveState() const
{
    return QByteArray();
}

void LUTGlDisplay::keyPressEvent( QKeyEvent * e )
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
            viewTranslation[0] = -64.0f;
            viewTranslation[1] = -64.0f;
            viewTranslation[2] = -300.0f;
            viewOrientation[0] = 0.0f;
            viewOrientation[1] = 0.0f;
            viewOrientation[2] = 0.0f;
            update();
            break;
        case Qt::Key_T:
            trueColours = !(trueColours);
            update();
            break;
        default:
            e->ignore();
            QGLWidget::keyPressEvent(e);
            break;
    }
    return;
}

void LUTGlDisplay::mousePressEvent ( QMouseEvent * event )
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

void LUTGlDisplay::mouseMoveEvent ( QMouseEvent * event )
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

void LUTGlDisplay::wheelEvent ( QWheelEvent * event )
{
    int magnitude = event->delta();
    float zoomDistance = (float)magnitude / 10.0f;
    viewTranslation[2] += zoomDistance;
    update();
}


void LUTGlDisplay::initializeGL()
{
    /*
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


    // Lighting
    glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbient);			// Setup The Ambient Light
    glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse);			// Setup The Diffuse Light
    glLightfv(GL_LIGHT1, GL_SPECULAR,LightSpecular);			// Setup Specular Light
    glLightfv(GL_LIGHT1, GL_POSITION,LightPosition);			// Position The Light

    glEnable(GL_LIGHT1);						// Enable Light One
    */
    quadratic = gluNewQuadric();
    gluQuadricNormals(quadratic, GLU_SMOOTH);	// Create Smooth Normals

    glEnable(GL_LIGHTING);      // Enable Global Lighting
}

void LUTGlDisplay::paintGL()
{
    makeCurrent();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);         // Clear The Screen And The Depth Buffer
    glLoadIdentity();						// Reset The Current Modelview Matrix


    glEnable(GL_COLOR_MATERIAL);                                // Enable Material Colour Tracking
    //GLfloat MatSpecular[]= { 1.0f, 1.0f, 1.0f, 1.0f };          // Diffuse Material Values
    GLfloat MatEmission[]= { 1.0f, 1.0f, 1.0f, 1.0f };  	// Diffuse Material Values

    glColorMaterial(GL_FRONT_AND_BACK, GL_EMISSION); // Set Ambient and Diffuse Material Values to Track
    //glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,MatSpecular);
    glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,MatEmission);
    //glMaterialf(GL_FRONT_AND_BACK,GL_SHININESS,100.0f);


    glTranslatef(viewTranslation[0],viewTranslation[1],viewTranslation[2]); // Move to centre of field position.
    glRotatef(viewOrientation[0],1.0f,0.0f,0.0f); // Rotate about X-Axis
    glRotatef(viewOrientation[1],0.0f,1.0f,0.0f); // Rotate about Y-Axis
    glRotatef(viewOrientation[2],0.0f,0.0f,1.0f); // Rotate about Z-Axis

    /*
    // Position Lighting
    GLfloat LightPosition[]= { 0.0f, 0.0f, 300.0f, 1.0f };      // Light Position
    glLightfv(GL_LIGHT1, GL_POSITION,LightPosition);		// Set Light Position
    */
   
    if(currentLUT)
    {
        DrawAxies();
        DrawLUT(currentLUT);
    }

    glFlush ();         // Run Queued Commands
    return;
}

void LUTGlDisplay::snapshotToClipboard()
{
    paintGL(); // Redraw the scene in case it needs to be updated.
    QClipboard *cb = QApplication::clipboard(); // get the clipboard
    QImage tempPicture(this->grabFrameBuffer(false)); // grab current image
    cb->setImage(tempPicture); // put current image on the clipboard.
}

void LUTGlDisplay::DrawAxies()
{
    // DRAW REFERENCE AXES
    glBegin(GL_LINES);
    glColor3f(1,0,0);
    glVertex3f(0,0,0);
    glVertex3f(128,0,0);

    glColor3f(0,1,0);
    glVertex3f(0,0,0);
    glVertex3f(0,128,0);

    glColor3f(0,0,1);
    glVertex3f(0,0,0);
    glVertex3f(0,0,128);
    glEnd();

    glDisable(GL_DEPTH_TEST);		// Turn Z Buffer testing Off
    glColor4ub(255,255,255,255);
    renderText(64, -10, -10, QString("Y"));
    renderText(-10, 64, -10, QString("Cb"));
    renderText(-10, -10, 64, QString("Cr"));
    glEnable(GL_DEPTH_TEST);		// Turn Z Buffer testing On
}

void LUTGlDisplay::DrawPoint(QColor colour, float y, float cb, float cr)
{

	glPushMatrix();


        glColor4ub(colour.red(),colour.green(),colour.blue(),colour.alpha());

        //float ballRadius = 0.5;
        //glTranslatef(x,y,z);                            // Move to centre of ball.
        //gluSphere(quadratic,ballRadius,4,4);		// Draw A Sphere
        glBegin(GL_POINTS);
        glVertex3f(y, cb, cr);
        glEnd();
        glColor4f(1.0f,1.0f,1.0f,1.0f); // Back to white
	glPopMatrix();
}

void LUTGlDisplay::DrawLUT(unsigned char* currentLUT)
{
	float y = 0;
	float cb = 0;
	float cr = 0;
	unsigned char r,g,b,a;
	//Go through each point on LUT and apply to 3d space
	for(int index = 0; index < LUTTools::LUT_SIZE; index++)
	{
            cr++;
            if (cr > 127)
            {
                cr = 0;
                cb++;
                if(cb > 127)
                {
                    cb = 0;
                    y++;
                }
            }


            //Do Not Draw Black Points:
            if(currentLUT[index] == ClassIndex::unclassified)
                    continue;

            a = 255;
            QColor colour;
            if(trueColours)
            {
                //Convert the YUV index to RGB Colour and display:
                ColorModelConversions::fromYCbCrToRGB((unsigned char)y*2,(unsigned char)cb*2,(unsigned char)cr*2,r,g,b);
                colour.setRed(r);
                colour.setGreen(g);
                colour.setBlue(b);
                colour.setAlpha(a);
            }
            else
            {
                ClassIndex::getColourIndexAsRGB(currentLUT[index],r,g,b);
                colour.setRed(r);
                colour.setGreen(g);
                colour.setBlue(b);
                colour.setAlpha(a);
            }
            DrawPoint(colour,y,cb,cr);
	}
}

void LUTGlDisplay::resizeGL(int width, int height)
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

QSize LUTGlDisplay::minimumSizeHint() const
{
        return QSize(50, 50);
}

QSize LUTGlDisplay::sizeHint() const
{
        return QSize(320, 240);
}
