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
#include "Infrastructure/NUImage/NUImage.h"
#include "Infrastructure/NUImage/ClassifiedImage.h"
#include "Kinematics/Horizon.h"
#include <QPainter>
#include <QDebug>

OpenglManager::OpenglManager(): width(320), height(240)
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
    makeCurrent();
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

void OpenglManager::writeNUImageToDisplay(const NUImage* newImage, GLDisplay::display displayId)
{
    width = newImage->getWidth();
    height = newImage->getHeight();

    QImage image(width,height,QImage::Format_ARGB32);
    unsigned char r, g, b;
    QRgb* imageLine;
    for (int y = 0; y < height; y++)
    {
        imageLine = (QRgb*)image.scanLine(y);
        for (int x = 0; x < width; x++)
        {
            Pixel pix = (*newImage)(x,y);
            ColorModelConversions::fromYCbCrToRGB(pix.y, pix.cb, pix.cr, r, g, b);
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
    makeCurrent();
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
    makeCurrent();
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
    makeCurrent();
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
    makeCurrent();
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
    makeCurrent();
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
    makeCurrent();
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

void OpenglManager::writeCalGridToDisplay(GLDisplay::display displayId)
{
    makeCurrent();
    // If there is an old list stored, delete it first.
    if(displayStored[displayId])
    {
        glDeleteLists(displays[displayId],1);
    }

    displays[displayId] = glGenLists(1);
    glNewList(displays[displayId],GL_COMPILE);    // START OF LIST
    glDisable(GL_TEXTURE_2D);


    drawHollowCircle(80, 60, 10, 50);
    drawHollowCircle(160, 60, 10, 50);
    drawHollowCircle(240, 60, 10, 50);

    drawHollowCircle(80, 120, 10, 50);
    drawHollowCircle(160, 120, 10, 50);
    drawHollowCircle(240, 120, 10, 50);

    drawHollowCircle(80, 180, 10, 50);
    drawHollowCircle(160, 180, 10, 50);
    drawHollowCircle(240, 180, 10, 50);

    glEnable(GL_TEXTURE_2D);
    glEndList();                                    // END OF LIST

    displayStored[displayId] = true;
    qDebug() << "Drawing Grid";
    emit updatedDisplay(displayId, displays[displayId], width, height);
    return;
}

void OpenglManager::clearDisplay(GLDisplay::display displayId)
{
    makeCurrent();
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
    makeCurrent();
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
    makeCurrent();
    int stepSize = 360 / num_segments;
    glBegin(GL_TRIANGLE_FAN);
    for(int angle = 0; angle < 360; angle += stepSize)
    {
        glVertex2f(cx + sinf(angle) * r, cy + cosf(angle) * r);
    }
    glEnd();
}
void OpenglManager::writeLinesPointsToDisplay(std::vector< LinePoint > linepoints, GLDisplay::display displayId)
{
    makeCurrent();
    //glDisable(GL_TEXTURE_2D);
    glLineWidth(1.0);
    glColor3ub(255,255,0);
    for(unsigned int i = 0; i < linepoints.size(); i++)
    {
        glBegin(GL_LINES);                              // Start Lines
        glVertex2i( int(linepoints[i].x-2), int(linepoints[i].y+2));                 // Starting point
        glVertex2i( int(linepoints[i].x+2), int(linepoints[i].y-2));               // Ending point
        glEnd();  // End Lines
        glBegin(GL_LINES);                              // Start Lines
        glVertex2i( int(linepoints[i].x+2), int(linepoints[i].y+2));                 // Starting point
        glVertex2i( int(linepoints[i].x-2), int(linepoints[i].y-2));               // Ending point
        glEnd();  // End Lines

    }
    //glEnable(GL_TEXTURE_2D);
    //glEndList();                                    // END OF LIST

    //displayStored[displayId] = true;

    //qDebug() << "Updating Linepoints:" << fieldLines.size();
    //emit updatedDisplay(displayId, displays[displayId], width, height);
}

void OpenglManager::writeFieldLinesToDisplay(std::vector< LSFittedLine > fieldLines, GLDisplay::display displayId)
{
    makeCurrent();
    // If there is an old list stored, delete it first.
    if(displayStored[displayId])
    {
        glDeleteLists(displays[displayId],1);
    }

    displays[displayId] = glGenLists(1);
    glNewList(displays[displayId],GL_COMPILE);    // START OF LIST
    glDisable(GL_TEXTURE_2D);

    glLineWidth(2.0);       // Line width
    

    for(unsigned int i = 0 ; i < fieldLines.size(); i++)
    {

        if(fieldLines[i].valid == true)
        {
            glLineWidth(3.0);       // Line width
            glColor3ub(255,0,0);
            glBegin(GL_LINES);                              // Start Lines
            glVertex2i( int(fieldLines[i].leftPoint.x), int(fieldLines[i].findYFromX(fieldLines[i].leftPoint.x)));                 // Starting point
            glVertex2i( int(fieldLines[i].rightPoint.x), int(fieldLines[i].findYFromX(fieldLines[i].rightPoint.x)));               // Ending point
            glEnd();  // End Lines

            /*qDebug()    << int(fieldLines[i].leftPoint.x) << "," << int(fieldLines[i].leftPoint.y) <<"\t"
                        << int(fieldLines[i].rightPoint.x)<< "," << int(fieldLines[i].rightPoint.y);*/

            std::vector<LinePoint*> linePoints = fieldLines[i].getPoints();
            glBegin(GL_TRIANGLES);
            for (unsigned int j =0; j < linePoints.size(); j++)
            {
                glVertex3f(int(linePoints[j]->x),int(linePoints[j]->y),0.0);
                glVertex3f(int(linePoints[j]->x),int(linePoints[j]->y-1),0.0);
                glVertex3f(int(linePoints[j]->x),int(linePoints[j]->y+1),0.0);
            }
            glEnd();
        }
        else
        {
            glLineWidth(2.0);       // Line width
            glColor3ub(100,0,50);
            glBegin(GL_LINES);                              // Start Lines
            glVertex2i( int(fieldLines[i].leftPoint.x), int(fieldLines[i].findYFromX(fieldLines[i].leftPoint.x)));                 // Starting point
            glVertex2i( int(fieldLines[i].rightPoint.x), int(fieldLines[i].findYFromX(fieldLines[i].rightPoint.x)));               // Ending point
            glEnd();  // End Lines

        }
    }
        //glEnable(GL_TEXTURE_2D);
        //glEndList();                                    // END OF LIST

        displayStored[displayId] = true;

        //qDebug() << "Updating FieldLines:" << fieldLines.size();
   // emit updatedDisplay(displayId, displays[displayId], width, height);

}
void OpenglManager::writeCornersToDisplay(std::vector< CornerPoint > corners, GLDisplay::display displayId)
{
    makeCurrent();
    glDisable(GL_TEXTURE_2D);
    glLineWidth(4.0);
    glColor3ub(255,0,255);
    for(unsigned int i = 0; i < corners.size(); i++)
    {
        glBegin(GL_LINES);                              // Start Lines
        glVertex2i( int(corners[i].PosX-4), int(corners[i].PosY+4));                 // Starting point
        glVertex2i( int(corners[i].PosX+4), int(corners[i].PosY-4));               // Ending point
        glEnd();  // End Lines
        glBegin(GL_LINES);                              // Start Lines
        glVertex2i( int(corners[i].PosX+4), int(corners[i].PosY+4));                 // Starting point
        glVertex2i( int(corners[i].PosX-4), int(corners[i].PosY-4));               // Ending point
        glEnd();  // End Lines
    }
    glEnable(GL_TEXTURE_2D);
    glEndList();                                    // END OF LIST

    displayStored[displayId] = true;

    qDebug() << "Updating Corner:" << corners.size();
    emit updatedDisplay(displayId, displays[displayId], width, height);
}

void OpenglManager::stub(QImage image, GLDisplay::display displayId)
{
    /*for (int x = 0; x < image.width(); x++)
    {
        for (int y = 0; y < image.height(); y++)
        {
            //image.setPixel(x,y, (0x00ffffff + ((x%256) << 24)));
            qDebug() << "p: ("<<x<<","<<y<<")" << image.pixel(x,y);
        }
    }//*/
    createDrawTextureImage(image, displayId);
    emit updatedDisplay(displayId, displays[displayId], width, height);
}

void OpenglManager::writeFieldObjectsToDisplay(FieldObjects* AllObjects, GLDisplay::display displayId)
{
    makeCurrent();
    //! CLEAR DRAWING LIST
    if(displayStored[displayId])
    {
        glDeleteLists(displays[displayId],1);
    }

    displays[displayId] = glGenLists(1);
    glNewList(displays[displayId],GL_COMPILE);    // START OF LIST
    glDisable(GL_TEXTURE_2D);
    glLineWidth(2.0);       // Line width

    //! DRAW STATIONARY OBJECTS:
    vector < StationaryObject > ::iterator statFOit;
    for(statFOit = AllObjects->stationaryFieldObjects.begin(); statFOit  < AllObjects->stationaryFieldObjects.end(); )
    {
        //! Check if the object is seen, if seen then continue to next Object
        if((*statFOit).isObjectVisible() == false)
        {
            ++statFOit;
            continue;
        }
        unsigned char r,g,b;
        if(     (*statFOit).getID() == FieldObjects::FO_BLUE_LEFT_GOALPOST  ||
                (*statFOit).getID() == FieldObjects::FO_BLUE_RIGHT_GOALPOST )
        {
            ClassIndex::getColourIndexAsRGB(ClassIndex::blue,r,g,b);
            glColor3ub(r,g,b);
        }
        else if(     (*statFOit).getID() == FieldObjects::FO_YELLOW_LEFT_GOALPOST ||
                     (*statFOit).getID() == FieldObjects::FO_YELLOW_RIGHT_GOALPOST )
        {
            ClassIndex::getColourIndexAsRGB(ClassIndex::yellow,r,g,b);
            glColor3ub(r,g,b);
        }
        else
        {
            ClassIndex::getColourIndexAsRGB(ClassIndex::white,r,g,b);
            glColor3ub(r,g,b);
        }

        if((*statFOit).getID() == FieldObjects::FO_CORNER_CENTRE_CIRCLE)
        {
            drawEllipse((*statFOit).ScreenX(),(*statFOit).ScreenY(), (*statFOit).getObjectWidth()/2, (*statFOit).getObjectHeight()/2);
            ++statFOit;
            continue;
        }

        int X = (*statFOit).ScreenX();
        int Y = (*statFOit).ScreenY();
        int ObjectWidth = (*statFOit).getObjectWidth();
        int ObjectHeight = (*statFOit).getObjectHeight();

        glBegin(GL_QUADS);                              // Start Lines
            glVertex2i( X-ObjectWidth/2, Y-ObjectHeight/2); //TOP LEFT
            glVertex2i( X+ObjectWidth/2, Y-ObjectHeight/2); //TOP RIGHT
            glVertex2i( X+ObjectWidth/2, Y+ObjectHeight/2); //BOTTOM RIGHT
            glVertex2i( X-ObjectWidth/2, Y+ObjectHeight/2); //BOTTOM LEFT
        glEnd();

        //! Incrememnt to next object:
        ++statFOit;
    }

    //! DRAW MOBILE OBJECTS:
    vector < MobileObject > ::iterator mobileFOit;
    for(mobileFOit = AllObjects->mobileFieldObjects.begin(); mobileFOit  < AllObjects->mobileFieldObjects.end(); )
    {
        //! Check if the object is seen, if seen then continue to next Object
        if((*mobileFOit).isObjectVisible() == false)
        {
            ++mobileFOit;
            continue;
        }
        qDebug() << "Seen: Mobile: " <<(*mobileFOit).getID() ;
        unsigned char r,g,b;
        //CHECK IF BALL: if so Draw a circle
        if(     (*mobileFOit).getID() == FieldObjects::FO_BALL)
        {
            ClassIndex::getColourIndexAsRGB(ClassIndex::orange,r,g,b);
            glColor3ub(r,g,b);

            int cx = (*mobileFOit).ScreenX();
            int cy = (*mobileFOit).ScreenY();
            int radius = (*mobileFOit).getObjectWidth()/2;
            int num_segments = 360;

            drawSolidCircle(cx, cy, radius, num_segments);
            ++mobileFOit;
            continue;
        }


        if(     (*mobileFOit).getID() == FieldObjects::FO_BLUE_ROBOT_1  ||
                (*mobileFOit).getID() == FieldObjects::FO_BLUE_ROBOT_2  ||
                (*mobileFOit).getID() == FieldObjects::FO_BLUE_ROBOT_3  ||
                (*mobileFOit).getID() == FieldObjects::FO_BLUE_ROBOT_4  )
        {
            ClassIndex::getColourIndexAsRGB(ClassIndex::shadow_blue,r,g,b);
            glColor3ub(r,g,b);
        }

        else if(     (*mobileFOit).getID() == FieldObjects::FO_PINK_ROBOT_1 ||
                     (*mobileFOit).getID() == FieldObjects::FO_PINK_ROBOT_2 ||
                     (*mobileFOit).getID() == FieldObjects::FO_PINK_ROBOT_3 ||
                     (*mobileFOit).getID() == FieldObjects::FO_PINK_ROBOT_4)
        {
            ClassIndex::getColourIndexAsRGB(ClassIndex::pink,r,g,b);
            glColor3ub(r,g,b);
        }

        int X = (*mobileFOit).ScreenX();
        int Y = (*mobileFOit).ScreenY();
        int ObjectWidth = (*mobileFOit).getObjectWidth();
        int ObjectHeight = (*mobileFOit).getObjectHeight();

        glBegin(GL_LINE_STRIP);                              // Start Lines
            glVertex2i( X-ObjectWidth/2, Y-ObjectHeight/2);
            glVertex2i( X-ObjectWidth/2, Y+ObjectHeight/2);
            glVertex2i( X+ObjectWidth/2, Y+ObjectHeight/2);
            glVertex2i( X+ObjectWidth/2, Y-ObjectHeight/2);
            glVertex2i( X-ObjectWidth/2, Y-ObjectHeight/2);
        glEnd();

        //! Increment to next object
        ++mobileFOit;
    }

    //! DRAW AMBIGUOUS OBJECTS: Using itterator as size is unknown

    vector < AmbiguousObject > ::iterator ambigFOit;
    qDebug() <<"Size Of Ambig Objects: " <<  AllObjects->ambiguousFieldObjects.size();
    for(ambigFOit = AllObjects->ambiguousFieldObjects.begin(); ambigFOit  < AllObjects->ambiguousFieldObjects.end(); )
    {
        //! Check if the object is seen, if seen then continue to next Object
        if((*ambigFOit).isObjectVisible() == false)
        {
            ++ambigFOit;
            continue;
        }
        qDebug() <<"Ambig Objects seen: " <<  (*ambigFOit).getID();
        unsigned char r,g,b;
        if(     (*ambigFOit).getID() == FieldObjects::FO_BLUE_ROBOT_UNKNOWN)
        {
            ClassIndex::getColourIndexAsRGB(ClassIndex::shadow_blue,r,g,b);
            glColor3ub(r,g,b);
        }

        else if(     (*ambigFOit).getID() == FieldObjects::FO_PINK_ROBOT_UNKNOWN)
        {
            ClassIndex::getColourIndexAsRGB(ClassIndex::pink,r,g,b);
            glColor3ub(r,g,b);
        }
        else if(     (*ambigFOit).getID() == FieldObjects::FO_BLUE_GOALPOST_UNKNOWN)
        {
            ClassIndex::getColourIndexAsRGB(ClassIndex::blue,r,g,b);
            glColor3ub(r,g,b);
        }
        else if(     (*ambigFOit).getID() == FieldObjects::FO_YELLOW_GOALPOST_UNKNOWN)
        {
            ClassIndex::getColourIndexAsRGB(ClassIndex::yellow,r,g,b);
            glColor3ub(r,g,b);
        }
        else
        {
            ClassIndex::getColourIndexAsRGB(ClassIndex::white,r,g,b);
            glColor3ub(r,g,b);

        }

        int X = (*ambigFOit).ScreenX();
        int Y = (*ambigFOit).ScreenY();
        int ObjectWidth = (*ambigFOit).getObjectWidth();
        int ObjectHeight = (*ambigFOit).getObjectHeight();

        glBegin(GL_LINE_STRIP);                              // Start Lines
            glVertex2i( X-ObjectWidth/2, Y-ObjectHeight/2);
            glVertex2i( X-ObjectWidth/2, Y+ObjectHeight/2);
            glVertex2i( X+ObjectWidth/2, Y+ObjectHeight/2);
            glVertex2i( X+ObjectWidth/2, Y-ObjectHeight/2);
            glVertex2i( X-ObjectWidth/2, Y-ObjectHeight/2);
        glEnd();

        //! Increment to next object
        ++ambigFOit;
    }


    //! UPDATE THE DISPLAY:
    glEnable(GL_TEXTURE_2D);
    glEndList();                                    // END OF LIST

    displayStored[displayId] = true;

    emit updatedDisplay(displayId, displays[displayId], width, height);
}

void OpenglManager::setExpectedVision(const NUSensorsData* data, const KF* filter, GLDisplay::display displayId)
{
    GLUquadricObj *qobj;
    makeCurrent();
    glDisable(GL_TEXTURE_2D);
    glLineWidth(4.0);
    glColor3ub(255,0,0);

    // Draw field and lines

    // Green field border
    glColor3ub(0,255,0);
    glBegin(GL_LINES);                      // Start Lines
    glVertex3i(-370, 270, 0);
    glVertex3i( 370, 270, 0);
    glVertex3i( 370,-270, 0);
    glVertex3i(-370,-270, 0);
    glEnd();  // End Lines

    // Outside field lines border
    glColor3ub(255,255,255);
    glBegin(GL_LINES);                      // Start Lines
    glVertex3i(-302.5, 202.5, 0);
    glVertex3i( 302.5, 202.5, 0);
    glVertex3i( 302.5,-202.5, 0);
    glVertex3i(-302.5,-202.5, 0);
    glEnd();  // End Lines

    glEnable(GL_TEXTURE_2D);
    glEndList();                                    // END OF LIST

    displayStored[displayId] = true;

    emit updatedDisplay(displayId, displays[displayId], width, height);

}

void  OpenglManager::drawEllipse(float cx, float cy, float xradius, float yradius)
{
    makeCurrent();
    glBegin(GL_LINE_STRIP);                              // Start Lines
        glVertex2i( cx-2, cy-2);
        glVertex2i( cx-2, cy+2);
        glVertex2i( cx+2, cy+2);
        glVertex2i( cx+2, cy-2);
        glVertex2i( cx-2, cy-2);
    glEnd();

   glBegin(GL_LINE_LOOP);

   for (int i=0; i < 360; i++)
   {
      //convert degrees into radians
      float degInRad = 3.14159/180 * i;
      glVertex2f(cx + cos(degInRad)*xradius, cy + sin(degInRad)*yradius);
   }

   glEnd();
}
