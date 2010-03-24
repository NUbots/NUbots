/*! @file openglmanager.cpp
    @brief Implementation of OpenglManager class.

    @author Steven Nicklin

  Copyright (c) 2010 Steven Nicklin

    This file is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This file is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with NUbot.  If not, see <http://www.gnu.org/licenses/>.
*/

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
        if(textureStored[id]) glDeleteTextures(1, &textures[id]);
        if(displayStored[id]) glDeleteLists(displays[id],1);
    }
}

void OpenglManager::createDrawTextureImage(const QImage& image, int displayId)
{
    // If there is a texture already stored, delete it.
    if(textureStored[displayId])
    {

        glDeleteTextures(1,&textures[displayId]);
        textureStored[displayId] = false;
    }

    QImage tex = QGLWidget::convertToGLFormat( image );
    glGenTextures( 1, &textures[displayId] );

    // Create Nearest Filtered Texture
    glBindTexture(GL_TEXTURE_2D, textures[displayId]);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, 4, tex.width(), tex.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, tex.bits());

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

void OpenglManager::writeNUimageToDisplay(const NUimage* newImage, GLDisplay::display displayId)
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

void OpenglManager::writeCandidatesToDisplay(std::vector< ObjectCandidate > candidates, GLDisplay::display displayId)
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
    std::vector<ObjectCandidate>::const_iterator i = candidates.begin();
    unsigned char r,g,b;
    for(; i != candidates.end(); i++)
    {
        ClassIndex::getColourIndexAsRGB(i->getColour(),r,g,b);
        Vector2<int> topLeft = i->getTopLeft();
        Vector2<int> bottomRight = i->getBottomRight();
        glColor3ub(r,g,b);

        glBegin(GL_LINE_STRIP);                              // Start Lines
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
void OpenglManager::writeWMLineToDisplay(WMLine* newWMLine, int numLines,GLDisplay::display displayId)
{
    // If there is an old list stored, delete it first.
    if(displayStored[displayId])
    {
        glDeleteLists(displays[displayId],1);
    }


    displays[displayId] = glGenLists(1);
    glNewList(displays[displayId],GL_COMPILE);    // START OF LIST
    glDisable(GL_TEXTURE_2D);

    glLineWidth(1.0);       // Line width
    glBegin(GL_LINES);                              // Start Lines
    for(int i = 0;i<numLines;i++)
    {
        glVertex2f(newWMLine[i].getStart().getx(),newWMLine[i].getStart().gety());                 // Starting point
        glVertex2f(newWMLine[i].getEnd().getx(),newWMLine[i].getEnd().gety());    // End point
    }
    glEnd();                                        // End Lines
    glEnable(GL_TEXTURE_2D);
    glEndList();                                    // END OF LIST


    displayStored[displayId] = true;
    emit updatedDisplay(displayId, displays[displayId], width, height);
    return;
}
void OpenglManager::writeWMBallToDisplay(float x, float y, float radius, GLDisplay::display displayId)
{

    // If there is an old list stored, delete it first.
    if(displayStored[displayId])
    {
        glDeleteLists(displays[displayId],1);
    }

    displays[displayId] = glGenLists(1);
    glNewList(displays[displayId],GL_COMPILE);    // START OF LIST
    glDisable(GL_TEXTURE_2D);





    drawHollowCircle(x, y, radius, 50);

    glEnable(GL_TEXTURE_2D);
    glEndList();                                    // END OF LIST

    displayStored[displayId] = true;

    emit updatedDisplay(displayId, displays[displayId], width, height);
    return;
}

void OpenglManager::clearDisplay(GLDisplay::display displayId)
{
    // If there is an old list stored, delete it first.
    if(displayStored[displayId])
    {
        glDeleteLists(displays[displayId],1);
    }
    displays[displayId] = glGenLists(1);
    emit updatedDisplay(displayId, displays[displayId], width, height);
    return;
}

void OpenglManager::clearAllDisplays()
{
    for (int disp = 0; disp < GLDisplay::numDisplays; disp++)
    {
        clearDisplay((GLDisplay::display)disp);
    }
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

void OpenglManager::writeFieldLinesToDisplay(std::vector< LSFittedLine > fieldLines, GLDisplay::display displayId)
{
    glLineWidth(2.0);       // Line width
    for(unsigned int i = 0 ; i < fieldLines.size(); i++)
    {

        if(fieldLines[i].valid == true){
            glBegin(GL_LINES);                              // Start Lines
            glVertex2i( int(fieldLines[i].leftPoint.x), int(fieldLines[i].leftPoint.y));                 // Starting point
            glVertex2i( int(fieldLines[i].rightPoint.x), int(fieldLines[i].rightPoint.y));                 // Starting point
            std::vector<LinePoint*> linePoints = fieldLines[i].getPoints();
            glEnd();  // End Lines
            glBegin(GL_TRIANGLES);
             for (unsigned int j =0; j < linePoints.size(); j++)
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
