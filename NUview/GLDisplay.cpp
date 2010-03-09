#include "GLDisplay.h"
#include "openglmanager.h"
#include <qclipboard.h>
#include <QApplication>
#include <QDataStream>

GLDisplay::GLDisplay(QWidget *parent, const OpenglManager * shareWidget):
        QGLWidget(parent,(QGLWidget*)shareWidget), imageWidth(80), imageHeight(60)
{
    for(int id = 0; id < numDisplays; id++)
    {
        overlays[id].displayID = id;
        overlays[id].primary = false;
        overlays[id].enabled = false;
        overlays[id].colour = getDefaultColour(id);
        overlays[id].hasDisplayCommand = shareWidget->hasDisplayCommand(id);
        overlays[id].displayCommand = shareWidget->getDisplayCommand(id);
    }
    imageWidth = shareWidget->getWidth();
    imageHeight = shareWidget->getHeight();
    primaryLayer = &overlays[0];
    setPrimaryDisplay(unknown);
    setMouseTracking(true);

    // Setup connections.
    connect(shareWidget,SIGNAL(updatedDisplay(int,GLuint,int,int)),this, SLOT(updatedDisplay(int, GLuint, int, int)));
    connect(this, SIGNAL(selectPixel(int,int)), parent ,SLOT(SelectColourAtPixel(int,int)));
    connect(this, SIGNAL(rightSelectPixel(int,int)), parent,SLOT(ClassifySelectedColour()));
    connect(this, SIGNAL(ctrlSelectPixel(int,int)), parent,SLOT(SelectAndClassifySelectedPixel(int,int)));
    return;
}

GLDisplay::~GLDisplay()
{
    return;
}

void GLDisplay::restoreState(const QByteArray & state)
{
    QDataStream stream(state);
    int primaryLayerIndex;
    int numDisplays;
    stream >> primaryLayerIndex;
    stream >> numDisplays;
    for(int i = 0; i < numDisplays; i++)
    {
        stream >> overlays[i].displayID;
        stream >> overlays[i].enabled;
        stream >> overlays[i].colour;
    }
    setPrimaryDisplay(primaryLayerIndex,overlays[primaryLayerIndex].colour);
    return;
}

QByteArray GLDisplay::saveState() const
{
    QByteArray state;
    QDataStream stream(&state,QIODevice::WriteOnly);
    stream << primaryLayer->displayID;
    stream << numDisplays;
    for(int i = 0; i < numDisplays; i++)
    {
        stream << overlays[i].displayID;
        stream << overlays[i].enabled;
        stream << overlays[i].colour;
    }
    return state;
}

void GLDisplay::initializeGL()
{
    glClearColor (0.0, 0.0, 0.0, 0.0);
    glDisable(GL_DEPTH_TEST);           // Disable Depth Testing

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0.0, imageWidth, imageHeight, 0.0, -1.0, 1.0);

    glEnable(GL_TEXTURE_2D);		// Enable Texture Mapping

    // Setup Blending
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    return;
}

void GLDisplay::updatedDisplay(int displayID, GLuint newDisplay, int width, int height)
{
    bool newSize = ((imageWidth != width) || (imageHeight != height));
    if( (primaryLayer->displayID == displayID) || (primaryLayer->displayID == unknown))
    {
        if(newSize)
        {
            imageWidth = width;
            imageHeight = height;
        }
    }
    overlays[displayID].displayCommand = newDisplay;
    overlays[displayID].hasDisplayCommand = true;
    updateGL();
}

void GLDisplay::setPrimaryDisplay(int displayID)
{
    return setPrimaryDisplay(displayID, getDefaultColour(displayID));
}

void GLDisplay::setPrimaryDisplay(int displayID, QColor drawingColour)
{
    primaryLayer->primary = false;
    primaryLayer->enabled = false;
    primaryLayer = &overlays[displayID];
    primaryLayer->primary = true;
    primaryLayer->enabled = true;
    primaryLayer->colour = drawingColour;
    setWindowTitle(getLayerName(displayID) + " Image");
    updateGL();
    return;
}

void GLDisplay::setOverlayDrawing(int displayID, bool enabled)
{
    overlays[displayID].enabled = enabled;;
}
void GLDisplay::setOverlayDrawing(int displayID, bool enabled, float alpha)
{
    overlays[displayID].enabled = enabled;
    overlays[displayID].colour.setAlpha((int)(alpha*255));
    return;
}

void GLDisplay::setOverlayDrawing(int displayID, bool enabled, QColor drawingColour)
{
    overlays[displayID].enabled = enabled;
    overlays[displayID].colour = drawingColour;
    return;
}

void GLDisplay::mousePressEvent(QMouseEvent * mouseEvent)
{
    QPoint pixel = calculateSelectedPixel(mouseEvent);

    bool badPixel = false;
    if((pixel.y() < 0) || (pixel.y() > imageHeight)) badPixel = true;
    if((pixel.x() < 0) || (pixel.x() > imageWidth)) badPixel = true;
    if(mouseEvent->button() == Qt::LeftButton)
    {
        Qt::KeyboardModifiers mod = mouseEvent->modifiers();
        /*
        if(mod & Qt::ShiftModifier)
        {
            emit shiftSelectPixel(mouseX,mouseY);
        }
        */
        if(mod & Qt::ControlModifier)
        {
            emit ctrlSelectPixel(pixel.x(),pixel.y());
        }
        else
        {
            emit selectPixel(pixel.x(),pixel.y());
        }
    }
    else if (mouseEvent->button() == Qt::RightButton)
    {
        emit rightSelectPixel(pixel.x(),pixel.y());
    }
    setMouseTracking(true);
    return;
}

void GLDisplay::mouseMoveEvent(QMouseEvent * mouseEvent)
{
    QPoint pixel = calculateSelectedPixel(mouseEvent);
    if((pixel.x() >=0) && (pixel.x() <= imageWidth) && (pixel.y() >= 0) && (pixel.y() <= imageHeight))
    {
        this->setCursor(Qt::CrossCursor);
    }
    else
    {
        this->setCursor(Qt::ArrowCursor);
    }
    setMouseTracking(true);
}

QPoint GLDisplay::calculateSelectedPixel(QMouseEvent * mouseEvent)
{
    int viewWidth = this->width();
    int viewHeight = this->height();

    int mouseX = mouseEvent->x();
    int mouseY = mouseEvent->y();

    //Rescaling the X and Y values to be between ImageWidth and ImageHeight
    int pixelX = mouseX * imageWidth/viewWidth;
    int pixelY = mouseY * imageHeight/viewHeight;
    return QPoint(pixelX,pixelY);
}

void GLDisplay::paintGL()
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0.0, imageWidth, imageHeight, 0.0, -1.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT);       // Clear The Screen

    if(primaryLayer->enabled && primaryLayer->hasDisplayCommand)
    {
        glColor4ub(primaryLayer->colour.red(),
                   primaryLayer->colour.green(),
                   primaryLayer->colour.blue(),
                   primaryLayer->colour.alpha());
        glCallList(primaryLayer->displayCommand);
    }
    else
    {
        return;
    }

    glEnable(GL_BLEND);		// Turn Blending On
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

    for (int id = 0; id < numDisplays; id++)
    {
        if(overlays[id].enabled && overlays[id].hasDisplayCommand && (overlays[id].primary == false))
        {
            glColor4ub(overlays[id].colour.red(),
                       overlays[id].colour.green(),
                       overlays[id].colour.blue(),
                       overlays[id].colour.alpha());
            glCallList(overlays[id].displayCommand);
            glColor4ub( 255, 255, 255, 255);
        }
    }
    glDisable(GL_BLEND);		// Turn Blending Off
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    glFlush();
    return;
}

void GLDisplay::resizeGL(int width, int height)
{
    glViewport(0, 0, width, height);
    updateGL();
    return;
}

QSize GLDisplay::minimumSizeHint() const
{
        return QSize(50, 50);
}

QSize GLDisplay::sizeHint() const
{
        return QSize(320, 240);
}

void GLDisplay::snapshotToClipboard()
{
    paintGL(); // Redraw the scene in case it needs to be updated.
    QClipboard *cb = QApplication::clipboard(); // get the clipboard
    QImage tempPicture(this->grabFrameBuffer(false)); // grab current image
    cb->setImage(tempPicture); // put current image on the clipboard.
}
