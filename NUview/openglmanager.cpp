#include "ColorModelConversions.h"
#include "Vision/ClassificationColours.h"
#include "openglmanager.h"
#include "Tools/Image/NUimage.h"
#include "Tools/Image/ClassifiedImage.h"
#include "Kinematics/Horizon.h"
#include <QPainter>

OpenglManager::OpenglManager(): width(0), height(0)
{
    for(int id = 0; id < GLDisplay::numDisplays; id++)
    {
        textureStored[id] = false;
        displayStored[id] = false;
    }
}

OpenglManager::~OpenglManager()
{
    for(int id = 0; id < GLDisplay::numDisplays; id++)
    {
        if(textureStored[id]) deleteTexture(textures[id]);
        if(displayStored[id]) glDeleteLists(displays[id],1);
    }
}

void OpenglManager::createDrawTextureImage(QImage& image, int displayId)
{
    // If there is a texture already stored, delete it.
    if(textureStored[displayId])
    {
        deleteTexture(textures[displayId]);
        textureStored[displayId] = false;
    }

    textures[displayId] = bindTexture(image, GL_TEXTURE_2D);
    textureStored[displayId] = true;

    // If there is an old list stored, delete it first.
    if(displayStored[displayId])
    {
        glDeleteLists(displays[displayId],1);
        displayStored[displayId] = false;
    }

    displays[displayId] = glGenLists(1);

    glNewList(displays[displayId],GL_COMPILE);
        glBindTexture(GL_TEXTURE_2D, textures[displayId]);
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST);    // Turn off filtering of textures
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);    // Turn off filtering of textures
        glBegin(GL_QUADS);
            glTexCoord2f(0.0f, 0.0f); glVertex3f(0.0f, (float)height,  1.0f);      // Bottom Left Of The Texture and Quad
            glTexCoord2f(1.0f, 0.0f); glVertex3f( (float)width, (float)height,  1.0f);    // Bottom Right Of The Texture and Quad
            glTexCoord2f(1.0f, 1.0f); glVertex3f( (float)width,  0.0f,  1.0f);     // Top Right Of The Texture and Quad
            glTexCoord2f(0.0f, 1.0f); glVertex3f(0.0f,  0.0f,  1.0f);       // Top Left Of The Texture and Quad
        glEnd();
    glEndList();
    displayStored[displayId] = true;
}

void OpenglManager::newRawImage(NUimage* newImage)
{
    width = newImage->width();
    height = newImage->height();
    QImage image(width,height,QImage::Format_ARGB32);
    unsigned char r, g, b;
    QRgb* imageLine;
    for (int y = 0; y < height; y++)
    {
        imageLine = (QRgb*)image.scanLine(y);
        for (int x = 0; x < width; x++)
        {
            ColorModelConversions::fromYCbCrToRGB(newImage->image[y][x].y,newImage->image[y][x].cb,newImage->image[y][x].cr,r,g,b);
            imageLine[x] = qRgb(r,g,b);
        }
    }
    createDrawTextureImage(image, GLDisplay::rawImage);
    emit updatedDisplay(GLDisplay::rawImage, displays[GLDisplay::rawImage], width, height);
    return;
}

void OpenglManager::newClassifiedImage(ClassifiedImage* newImage)
{
    width = newImage->width();
    height = newImage->height();
    QImage image(width,height,QImage::Format_ARGB32);

    unsigned char r, g, b, alpha;
    QRgb* imageLine;
    int tempIndex;
    for (int y=0; y < height; y++)
    {
        imageLine = (QRgb*)image.scanLine(y);
        for (int x=0; x < width; x++)
        {
            alpha = 255;
            tempIndex = newImage->image[y][x];
            if(tempIndex == ClassIndex::unclassified) alpha = 0;
            ClassIndex::getColourIndexAsRGB(tempIndex, r, g, b);
            imageLine[x] = qRgba(r,g,b,alpha);
        }
    }
    createDrawTextureImage(image, GLDisplay::classifiedImage);
    emit updatedDisplay(GLDisplay::classifiedImage, displays[GLDisplay::classifiedImage], width, height);
    return;
}

void OpenglManager::newHorizon(Horizon* newHorizon)
{
    // If there is an old list stored, delete it first.
    if(displayStored[GLDisplay::horizonLine])
    {
        glDeleteLists(displays[GLDisplay::horizonLine],1);
    }

    displays[GLDisplay::horizonLine] = glGenLists(1);
    glNewList(displays[GLDisplay::horizonLine],GL_COMPILE);    // START OF LIST
    glDisable(GL_TEXTURE_2D);

    glLineWidth(2.0);       // Line width
    glBegin(GL_LINES);                              // Start Lines
    glVertex2i( 0, (int)newHorizon->findYFromX(0));                 // Starting point
    glVertex2i( (int)width, (int)newHorizon->findYFromX(width));    // End point
    glEnd();                                        // End Lines
    glEnable(GL_TEXTURE_2D);
    glEndList();                                    // END OF LIST

    displayStored[GLDisplay::horizonLine] = true;

    emit updatedDisplay(GLDisplay::horizonLine, displays[GLDisplay::horizonLine], width, height);
    return;
}

void OpenglManager::newClassificationSelection(ClassifiedImage* newImage)
{
    width = newImage->width();
    height = newImage->height();
    QImage image(width,height,QImage::Format_ARGB32);

    unsigned char r, g, b, alpha;
    QRgb* imageLine;
    int tempIndex;
    for (int y=0; y < height; y++)
    {
        imageLine = (QRgb*)image.scanLine(y);
        for (int x=0; x < width; x++)
        {
            alpha = 255;
            tempIndex = newImage->image[y][x];
            if(tempIndex == ClassIndex::unclassified) alpha = 0;
            ClassIndex::getColourIndexAsRGB(tempIndex, r, g, b);
            imageLine[x] = qRgba(r,g,b,alpha);
        }
    }
    createDrawTextureImage(image, GLDisplay::classificationSelection);
    emit updatedDisplay(GLDisplay::classificationSelection, displays[GLDisplay::classificationSelection], width, height);
    return;
}

void OpenglManager::newGreenpoints(std::vector< Vector2<int> > newpoints)
{
    // If there is an old list stored, delete it first.
    if(displayStored[GLDisplay::greenHorizonScanPoints])
    {
        glDeleteLists(displays[GLDisplay::greenHorizonScanPoints],1);
    }

    displays[GLDisplay::greenHorizonScanPoints] = glGenLists(1);
    glNewList(displays[GLDisplay::greenHorizonScanPoints],GL_COMPILE);    // START OF LIST
    glDisable(GL_TEXTURE_2D);
    for (int pointNum = 0; pointNum < (int)newpoints.size(); pointNum++)
    {
        drawHollowCircle(newpoints[pointNum].x+0.5, newpoints[pointNum].y+0.5, 0.5, 50);
    }
    glEnable(GL_TEXTURE_2D);
    glEndList();                                    // END OF LIST

    displayStored[GLDisplay::greenHorizonScanPoints] = true;

    emit updatedDisplay(GLDisplay::greenHorizonScanPoints, displays[GLDisplay::greenHorizonScanPoints], width, height);
    return;
}

void OpenglManager::drawHollowCircle(float cx, float cy, float r, int num_segments)
{
    int stepSize = 360 / num_segments;
    glBegin(GL_LINE_LOOP);
    for(int angle = 0; angle < 360; angle += stepSize)
    {
        glVertex2f(cx + sinf(angle) * r, cy + cosf(angle) * r);
    }
    glEnd();
}

void OpenglManager::drawSolidCircle(float cx, float cy, float r, int num_segments)
{
    int stepSize = 360 / num_segments;
    glBegin(GL_TRIANGLE_FAN);
    for(int angle = 0; angle < 360; angle += stepSize)
    {
        glVertex2f(cx + sinf(angle) * r, cy + cosf(angle) * r);
    }
    glEnd();
}
