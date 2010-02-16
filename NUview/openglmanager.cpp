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

void OpenglManager::writeNUimageToDisplay(NUimage* newImage, GLDisplay::display displayId)
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
    createDrawTextureImage(image, displayId);
    emit updatedDisplay(displayId, displays[displayId], width, height);
    return;
}

void OpenglManager::writeClassImageToDisplay(ClassifiedImage* newImage, GLDisplay::display displayId)
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
    createDrawTextureImage(image, displayId);
    emit updatedDisplay(displayId, displays[displayId], width, height);
    return;
}

void OpenglManager::writeLineToDisplay(Line* newLine, GLDisplay::display displayId)
{
    // If there is an old list stored, delete it first.
    if(displayStored[displayId])
    {
        glDeleteLists(displays[displayId],1);
    }

    displays[displayId] = glGenLists(1);
    glNewList(displays[displayId],GL_COMPILE);    // START OF LIST
    glDisable(GL_TEXTURE_2D);

    glLineWidth(2.0);       // Line width
    glBegin(GL_LINES);                              // Start Lines
    glVertex2i( 0, (int)newLine->findYFromX(0));                 // Starting point
    glVertex2i( (int)width, (int)newLine->findYFromX(width));    // End point
    glEnd();                                        // End Lines
    glEnable(GL_TEXTURE_2D);
    glEndList();                                    // END OF LIST

    displayStored[displayId] = true;

    emit updatedDisplay(displayId, displays[displayId], width, height);
    return;
}

void OpenglManager::writePointsToDisplay(std::vector< Vector2<int> > newpoints, GLDisplay::display displayId)
{
    // If there is an old list stored, delete it first.
    if(displayStored[displayId])
    {
        glDeleteLists(displays[displayId],1);
    }

    displays[displayId] = glGenLists(1);
    glNewList(displays[displayId],GL_COMPILE);    // START OF LIST
    glDisable(GL_TEXTURE_2D);
    for (int pointNum = 0; pointNum < (int)newpoints.size(); pointNum++)
    {
        drawHollowCircle(newpoints[pointNum].x+0.5, newpoints[pointNum].y+0.5, 0.5, 50);
    }
    glEnable(GL_TEXTURE_2D);
    glEndList();                                    // END OF LIST

    displayStored[displayId] = true;

    emit updatedDisplay(displayId, displays[displayId], width, height);
    return;
}

void OpenglManager::writeTransitionSegmentsToDisplay(std::vector< TransitionSegment > newsegments, GLDisplay::display displayId)
{
    // If there is an old list stored, delete it first.
    if(displayStored[displayId])
    {
        glDeleteLists(displays[displayId],1);
    }

    displays[displayId] = glGenLists(1);
    glNewList(displays[displayId],GL_COMPILE);    // START OF LIST
    glDisable(GL_TEXTURE_2D);
    glLineWidth(2.0);       // Line width
    std::vector<TransitionSegment>::const_iterator i;
    unsigned char r,g,b;
    for(i = newsegments.begin(); i != newsegments.end(); i++)
    {
        Vector2<int> start = i->getStartPoint();
        Vector2<int> end =i->getEndPoint();
        ClassIndex::getColourIndexAsRGB(i->getColour(),r,g,b);
        glColor3ub(r,g,b);

        glBegin(GL_LINES);                              // Start Lines
        glVertex2i( start.x, start.y);                 // Starting point
        glVertex2i( end.x, end.y);                 // Starting point
        glEnd();                                        // End Lines
    }
    glEnable(GL_TEXTURE_2D);
    glEndList();                                    // END OF LIST

    displayStored[displayId] = true;

    emit updatedDisplay(displayId, displays[displayId], width, height);
}

void OpenglManager::writeRobotCandidatesToDisplay(std::vector< RobotCandidate > robotCandidates, GLDisplay::display displayId)
{

    // If there is an old list stored, delete it first.
    if(displayStored[displayId])
    {
        glDeleteLists(displays[displayId],1);
    }

    displays[displayId] = glGenLists(1);
    glNewList(displays[displayId],GL_COMPILE);    // START OF LIST
    glDisable(GL_TEXTURE_2D);
    glLineWidth(2.0);       // Line width
    std::vector<RobotCandidate>::const_iterator i;
    unsigned char r,g,b;
    for(i = robotCandidates.begin(); i != robotCandidates.end(); i++)
    {
        Vector2<int> topLeft = i->getTopLeft();
        Vector2<int> bottomRight = i->getBottomRight();
        ClassIndex::getColourIndexAsRGB(i->getTeamColour(),r,g,b);
        glColor3ub(r,g,b);

        glBegin(GL_LINE_LOOP);                              // Start Lines
            glVertex2i( topLeft.x, topLeft.y);
            glVertex2i( topLeft.x, bottomRight.y);
            glVertex2i( bottomRight.x, bottomRight.y);
            glVertex2i( bottomRight.x, topLeft.y);
        glEnd();                                        // End Lines
    }
    glEnable(GL_TEXTURE_2D);
    glEndList();                                    // END OF LIST

    displayStored[displayId] = true;

    emit updatedDisplay(displayId, displays[displayId], width, height);

}

void OpenglManager::writeFieldLinesToDisplay(std::vector< LSFittedLine > fieldLines, GLDisplay::display displayId)
{

    // If there is an old list stored, delete it first.
    if(displayStored[displayId])
    {
        glDeleteLists(displays[displayId],1);
    }

    displays[displayId] = glGenLists(1);
    glNewList(displays[displayId],GL_COMPILE);    // START OF LIST
    glDisable(GL_TEXTURE_2D);
    glLineWidth(2.0);       // Line width
    //std::vector<RobotCandidate>::const_iterator i;
    unsigned char r,g,b;
    for(int i = 0 ; i < fieldLines.size(); i++)
    {
        //Vector2<int> topLeft = i->getTopLeft();
        //Vector2<int> bottomRight = i->getBottomRight();
        //ClassIndex::getColourIndexAsRGB(i->getTeamColour(),r,g,b);
        //glColor3ub(255,255,255);
        if(fieldLines[i].valid == true){
            glBegin(GL_LINES);                              // Start Lines
            glVertex2i( int(fieldLines[i].leftPoint.x), int(fieldLines[i].leftPoint.y));                 // Starting point
            glVertex2i( int(fieldLines[i].rightPoint.x), int(fieldLines[i].rightPoint.y));                 // Starting point
            std::vector<LinePoint*> linePoints = fieldLines[i].getPoints();
            glEnd();  // End Lines
            glBegin(GL_TRIANGLES);
             for (int j =0; j < linePoints.size(); j++)
            {
                glVertex3f(int(linePoints[j]->x),int(linePoints[j]->y),0.0);
                glVertex3f(int(linePoints[j]->x),int(linePoints[j]->y+1),0.0);
                glVertex3f(int(linePoints[j]->x-1),int(linePoints[j]->y+0.5),0.0);
            }
            glEnd();
        }
    }
        glEnable(GL_TEXTURE_2D);
        glEndList();                                    // END OF LIST

        displayStored[displayId] = true;


    emit updatedDisplay(displayId, displays[displayId], width, height);

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
