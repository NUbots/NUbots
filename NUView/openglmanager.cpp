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
#include "Vision/VisionTools/classificationcolours.h"
#include "openglmanager.h"
#include "Infrastructure/NUImage/NUImage.h"
#include "Infrastructure/NUImage/ClassifiedImage.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Kinematics/Horizon.h"
#include <QPainter>
#include <QDebug>
#include <QGLPixelBuffer>
#include "SensorCalibrationWidget.h"

// Apple has to be different...
#if defined(__APPLE__) || defined(MACOSX)
  #include <glu.h>
#else
  #include <GL/glu.h>
#endif

#include <boost/foreach.hpp>

OpenglManager::OpenglManager(): width(640), height(480)
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
            const Pixel& pix = (*newImage)(x,y);
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
            if(tempIndex == Vision::unclassified) alpha = 0;
            Vision::getColourAsRGB(Vision::getColourFromIndex(tempIndex), r, g, b);
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

void OpenglManager::writePointsToDisplay(std::vector<Point> newpoints, GLDisplay::display displayId)
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

void OpenglManager::writeSegmentsToDisplay(vector<vector<ColourSegment> > updatedSegments, GLDisplay::display displayId)
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
    for(unsigned int i = 0 ; i < updatedSegments.size(); i++)
    {
        vector<ColourSegment>& line = updatedSegments[i];
        for(unsigned int k = 0 ; k < line.size(); k++) {
            ColourSegment& segment = line[i];
            const Point& s = segment.getStart();
            const Point& e = segment.getEnd();
            unsigned char r, g, b;


            Vision::getColourAsRGB(segment.getColour(), r, g, b);
            glColor3ub(r,g,b);
            glBegin(GL_LINES);                              // Start Lines
            glVertex2i( int(s.x), int(s.y) );                 // Starting point
            glVertex2i( int(e.x), int(e.y) );               // Ending point
            glEnd();  // End Lines
        }
    }
    glEnable(GL_TEXTURE_2D);
    glEndList();                                    // END OF LIST

    displayStored[displayId] = true;

    emit updatedDisplay(displayId, displays[displayId], width, height);
    return;
}

//void OpenglManager::writeTransitionSegmentsToDisplay(std::vector< TransitionSegment > newsegments, GLDisplay::display displayId)
//{
//    makeCurrent();
//    // If there is an old list stored, delete it first.
//    if(displayStored[displayId])
//    {
//        glDeleteLists(displays[displayId],1);
//    }

//    displays[displayId] = glGenLists(1);
//    glNewList(displays[displayId],GL_COMPILE);    // START OF LIST
//    glDisable(GL_TEXTURE_2D);
//    glLineWidth(2.0);       // Line width
//    std::vector<TransitionSegment>::const_iterator i;
//    unsigned char r,g,b;
//    for(i = newsegments.begin(); i != newsegments.end(); i++)
//    {
//        Vector2<int> start = i->getStartPoint();
//        Vector2<int> end =i->getEndPoint();
//        Vision::getColourIndexAsRGB(i->getColour(),r,g,b);
//        glColor3ub(r,g,b);

//        glBegin(GL_LINES);                              // Start Lines
//        glVertex2i( start.x, start.y);                 // Starting point
//        glVertex2i( end.x, end.y);                 // Starting point
//        glEnd();                                        // End Lines
//    }
//    glEnable(GL_TEXTURE_2D);
//    glEndList();                                    // END OF LIST

//    displayStored[displayId] = true;

//    emit updatedDisplay(displayId, displays[displayId], width, height);
//}

//void OpenglManager::writeCandidatesToDisplay(std::vector< ObjectCandidate > candidates, GLDisplay::display displayId)
//{
//    makeCurrent();
//    // If there is an old list stored, delete it first.
//    if(displayStored[displayId])
//    {
//        glDeleteLists(displays[displayId],1);
//    }

//    displays[displayId] = glGenLists(1);
//    glNewList(displays[displayId],GL_COMPILE);    // START OF LIST
//    glDisable(GL_TEXTURE_2D);
//    glLineWidth(2.0);       // Line width
//    std::vector<ObjectCandidate>::const_iterator i = candidates.begin();
//    unsigned char r,g,b;
//    for(; i != candidates.end(); i++)
//    {
//        Vision::getColourIndexAsRGB(i->getColour(),r,g,b);
//        Vector2<int> topLeft = i->getTopLeft();
//        Vector2<int> bottomRight = i->getBottomRight();
//        glColor3ub(r,g,b);

//        glBegin(GL_LINE_STRIP);                              // Start Lines
//            glVertex2i( topLeft.x, topLeft.y);
//            glVertex2i( topLeft.x, bottomRight.y);
//            glVertex2i( bottomRight.x, bottomRight.y);
//            glVertex2i( bottomRight.x, topLeft.y);
//        glEnd();                                        // End Lines

//    }
//    glEnable(GL_TEXTURE_2D);
//    glEndList();                                    // END OF LIST

//    displayStored[displayId] = true;

//    emit updatedDisplay(displayId, displays[displayId], width, height);

//}

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
    DrawArc(cx, cy, r, 0, 360, num_segments);
    return;
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
void OpenglManager::writeLinesPointsToDisplay(vector<Point> linepoints, GLDisplay::display displayId)
{
    makeCurrent();
    glDisable(GL_TEXTURE_2D);
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
    glEnable(GL_TEXTURE_2D);
    glEndList();                                    // END OF LIST

    displayStored[displayId] = true;

//    qDebug() << "Updating Linepoints:" << fieldLines.size();
    emit updatedDisplay(displayId, displays[displayId], width, height);
}

void OpenglManager::writeLinesToDisplay(std::vector< LSFittedLine > lines, GLDisplay::display displayId)
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
    

    for(unsigned int i = 0 ; i < lines.size(); i++)
    {
        const LSFittedLine& line = lines[i];
        if(line.valid == true)
        {
            Vector2<double> ep1, ep2;
            line.getEndPoints(ep1, ep2);

            glLineWidth(3.0);       // Line width

            switch(displayId) {
            case GLDisplay::GoalEdgeLinesStart:
                glColor3ub(0,255,255);  //cyan
                break;
            case GLDisplay::GoalEdgeLinesEnd:
                glColor3ub(255,0,255);    //magenta
                break;
            default:
                glColor3ub(255,0,0);    //red
            }



            glBegin(GL_LINES);                              // Start Lines
            glVertex2i( int(ep1.x), int(ep1.y) );                 // Starting point
            glVertex2i( int(ep2.x), int(ep2.y) );               // Ending point
            glEnd();  // End Lines

            const std::vector< Vector2<double> >& linePoints = line.getPoints();
            glBegin(GL_TRIANGLES);
            for (unsigned int j =0; j < linePoints.size(); j++)
            {
                glVertex3f(int(linePoints[j].x),int(linePoints[j].y),0.0);
                glVertex3f(int(linePoints[j].x),int(linePoints[j].y-1),0.0);
                glVertex3f(int(linePoints[j].x),int(linePoints[j].y+1),0.0);
            }
            glEnd();
        }
        //HOW CAN WE RENDER AN INVALID LINE?
//        else
//        {
//            glLineWidth(2.0);       // Line width
//            glColor3ub(100,0,50);
//            glBegin(GL_LINES);                              // Start Lines
//            glVertex2i( int(ep1.x), int(ep1.y) );                 // Starting point
//            glVertex2i( int(ep2.x), int(ep2.y) );               // Ending point
//            glEnd();  // End Lines
//        }
    }
        //glEnable(GL_TEXTURE_2D);
        //glEndList();                                    // END OF LIST

        displayStored[displayId] = true;

        //qDebug() << "Updating FieldLines:" << fieldLines.size();
   // emit updatedDisplay(displayId, displays[displayId], width, height);

}
//void OpenglManager::writeCornersToDisplay(std::vector< CornerPoint > corners, GLDisplay::display displayId)
//{
//    makeCurrent();
//    glDisable(GL_TEXTURE_2D);
//    glLineWidth(4.0);
//    glColor3ub(255,0,255);
//    for(unsigned int i = 0; i < corners.size(); i++)
//    {
//        glBegin(GL_LINES);                              // Start Lines
//        glVertex2i( int(corners[i].PosX-4), int(corners[i].PosY+4));                 // Starting point
//        glVertex2i( int(corners[i].PosX+4), int(corners[i].PosY-4));               // Ending point
//        glEnd();  // End Lines
//        glBegin(GL_LINES);                              // Start Lines
//        glVertex2i( int(corners[i].PosX+4), int(corners[i].PosY+4));                 // Starting point
//        glVertex2i( int(corners[i].PosX-4), int(corners[i].PosY-4));               // Ending point
//        glEnd();  // End Lines
//    }
//    glEnable(GL_TEXTURE_2D);
//    glEndList();                                    // END OF LIST

//    displayStored[displayId] = true;

//    qDebug() << "Updating Corner:" << corners.size();
//    emit updatedDisplay(displayId, displays[displayId], width, height);
//}

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
            Vision::getColourAsRGB(Vision::blue,r,g,b);
            glColor3ub(r,g,b);
        }
        else if(     (*statFOit).getID() == FieldObjects::FO_YELLOW_LEFT_GOALPOST ||
                     (*statFOit).getID() == FieldObjects::FO_YELLOW_RIGHT_GOALPOST )
        {
            Vision::getColourAsRGB(Vision::yellow,r,g,b);
            glColor3ub(r,g,b);
        }
        else
        {
            Vision::getColourAsRGB(Vision::white,r,g,b);
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
            Vision::getColourAsRGB(Vision::orange,r,g,b);
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
            Vision::getColourAsRGB(Vision::shadow_blue,r,g,b);
            glColor3ub(r,g,b);
        }

        else if(     (*mobileFOit).getID() == FieldObjects::FO_PINK_ROBOT_1 ||
                     (*mobileFOit).getID() == FieldObjects::FO_PINK_ROBOT_2 ||
                     (*mobileFOit).getID() == FieldObjects::FO_PINK_ROBOT_3 ||
                     (*mobileFOit).getID() == FieldObjects::FO_PINK_ROBOT_4)
        {
            Vision::getColourAsRGB(Vision::pink,r,g,b);
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
            Vision::getColourAsRGB(Vision::shadow_blue,r,g,b);
            glColor3ub(r,g,b);
        }

        else if(     (*ambigFOit).getID() == FieldObjects::FO_PINK_ROBOT_UNKNOWN)
        {
            Vision::getColourAsRGB(Vision::pink,r,g,b);
            glColor3ub(r,g,b);
        }
        else if(     (*ambigFOit).getID() == FieldObjects::FO_BLUE_GOALPOST_UNKNOWN)
        {
            Vision::getColourAsRGB(Vision::blue,r,g,b);
            glColor3ub(r,g,b);
        }
        else if(     (*ambigFOit).getID() == FieldObjects::FO_YELLOW_GOALPOST_UNKNOWN)
        {
            Vision::getColourAsRGB(Vision::yellow,r,g,b);
            glColor3ub(r,g,b);
        }
        else
        {
            Vision::getColourAsRGB(Vision::white,r,g,b);
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

void OpenglManager::DrawArc(float cx, float cy, float r, float start_angle, float arc_angle, int num_segments)
{
    float theta = arc_angle / float(num_segments - 1);//theta is now calculated from the arc angle instead, the - 1 bit comes from the fact that the arc is open

    float tangetial_factor = tanf(theta);

    float radial_factor = cosf(theta);


    float x = r * cosf(start_angle);//we now start at the start angle
    float y = r * sinf(start_angle);

    glBegin(GL_LINE_STRIP);//since the arc is not a closed curve, this is a strip now
    for(int ii = 0; ii < num_segments; ii++)
    {
        glVertex2f(x + cx, y + cy);

        float tx = -y;
        float ty = x;

        x += tx * tangetial_factor;
        y += ty * tangetial_factor;

        x *= radial_factor;
        y *= radial_factor;
    }
    glEnd();
}

void OpenglManager::writeExpectedViewToDisplay(const NUSensorsData* SensorData, SensorCalibration* calibration, GLDisplay::display displayId)
{
    const float fov_horizontal = 60.0;
    const float fov_vertical = 46.0;

    // Make field - all done manually unfortunately
    typedef Vector2<float> point;
    typedef std::pair<point, point> line;
    std::vector<line> field_lines;

    // Field dimensions in cm.
    const float line_width = 5.f;
    const float field_width = 400.f;
    const float field_length = 600.f;
    const float penalty_width = 220.f;
    const float penalty_length = 60.f;
    const float pen_spot_distance = 180.f;
    const float pen_spot_length = 10.f;
    const float center_circle_diameter = 120.f;

    // Other measurements calculated from field dimensions.
    const float lhw = 0.5f * line_width;        // line half width
    const float hfw = 0.5f * field_width;       // half field width
    const float hfl = 0.5f * field_length;      // half field length
    const float hpw = 0.5f * penalty_width;     // half penalty width
    const float hpl = 0.5f * penalty_length;    // half penalty length
    const float center_circle_radius = 0.5f * center_circle_diameter;

    // Base and side line positions.
    const float inner_base = hfl - lhw;
    const float outer_base = hfl + lhw;
    const float inner_side = hfw - lhw;
    const float outer_side = hfw + lhw;

    // border
    field_lines.push_back(line(point(-outer_base,-outer_side),point(-outer_base, outer_side))); // Blue baseline
    field_lines.push_back(line(point( outer_base,-outer_side),point( outer_base, outer_side))); // Yellow baseline
    field_lines.push_back(line(point(-outer_base, outer_side),point( outer_base, outer_side))); // Sideline
    field_lines.push_back(line(point(-outer_base,-outer_side),point( outer_base,-outer_side))); // Sideline

    // penalty box positions.
    const float inner_penalty_side = hpw - lhw;
    const float outer_penalty_side = hpw + lhw;
    const float inner_penalty_front = hfl - penalty_length + lhw;
    const float outer_penalty_front = hfl - penalty_length - lhw;

    // Inner baseline Blue
    field_lines.push_back(line(point(-inner_base,-inner_side),point(-inner_base,-outer_penalty_side))); // Side
    field_lines.push_back(line(point(-inner_base, inner_side),point(-inner_base, outer_penalty_side))); // Side
    field_lines.push_back(line(point(-inner_base,-inner_penalty_side),point(-inner_base, inner_penalty_side))); // centre

    // Inner baseline Yellow
    field_lines.push_back(line(point( inner_base,-inner_side),point( inner_base,-outer_penalty_side))); // Side
    field_lines.push_back(line(point( inner_base, inner_side),point( inner_base, outer_penalty_side))); // Side
    field_lines.push_back(line(point( inner_base,-inner_penalty_side),point( inner_base, inner_penalty_side))); // centre

    // Inner Sidelines
    field_lines.push_back(line(point(-inner_base, inner_side),point(-lhw, inner_side))); // Sideline
    field_lines.push_back(line(point( inner_base, inner_side),point( lhw, inner_side))); // Sideline
    field_lines.push_back(line(point(-inner_base,-inner_side),point(-lhw,-inner_side))); // Sideline
    field_lines.push_back(line(point( inner_base,-inner_side),point( lhw,-inner_side))); // Sideline

    // Blue Penalty Box
    field_lines.push_back(line(point(-inner_base,-inner_penalty_side),point(-inner_penalty_front,-inner_penalty_side))); // Side inner
    field_lines.push_back(line(point(-inner_base,-outer_penalty_side),point(-outer_penalty_front,-outer_penalty_side))); // Side outer
    field_lines.push_back(line(point(-inner_base, inner_penalty_side),point(-inner_penalty_front, inner_penalty_side))); // Side inner
    field_lines.push_back(line(point(-inner_base, outer_penalty_side),point(-outer_penalty_front, outer_penalty_side))); // Side outer
    field_lines.push_back(line(point(-inner_penalty_front,-inner_penalty_side),point(-inner_penalty_front, inner_penalty_side))); // Front inner
    field_lines.push_back(line(point(-outer_penalty_front,-outer_penalty_side),point(-outer_penalty_front, outer_penalty_side))); // Front outer

    // Yellow Penalty Box
    field_lines.push_back(line(point( inner_base,-inner_penalty_side),point( inner_penalty_front,-inner_penalty_side))); // Side inner
    field_lines.push_back(line(point( inner_base,-outer_penalty_side),point( outer_penalty_front,-outer_penalty_side))); // Side outer
    field_lines.push_back(line(point( inner_base, inner_penalty_side),point( inner_penalty_front, inner_penalty_side))); // Side inner
    field_lines.push_back(line(point( inner_base, outer_penalty_side),point( outer_penalty_front, outer_penalty_side))); // Side outer
    field_lines.push_back(line(point( inner_penalty_front,-inner_penalty_side),point( inner_penalty_front, inner_penalty_side))); // Front inner
    field_lines.push_back(line(point( outer_penalty_front,-outer_penalty_side),point( outer_penalty_front, outer_penalty_side))); // Front outer

    // Center circle calculations.
    const float inner_cc = center_circle_radius - lhw;
    const float outer_cc = center_circle_radius + lhw;
    const float inner_arc_start_angle = asin(lhw / inner_cc);   // work out where the inner center circle meets the edge of the halfway line.
    const float outer_arc_start_angle = asin(lhw / outer_cc);   // work out where the outer center circle meets the edge of the halfway line.

    // Penalty spot positions.
    const float spot_hl = 0.5*pen_spot_length;
    const float spot_center = hfl - pen_spot_distance;
    const float spot_front = spot_center - spot_hl;
    const float spot_back = spot_center + spot_hl;

    // Halfway line
    field_lines.push_back(line(point(-lhw, inner_side),point(-lhw, outer_cc)));
    field_lines.push_back(line(point( lhw, inner_side),point( lhw, outer_cc)));
    field_lines.push_back(line(point(-lhw,-inner_side),point(-lhw,-outer_cc)));
    field_lines.push_back(line(point( lhw,-inner_side),point( lhw,-outer_cc)));
    field_lines.push_back(line(point(-lhw,-inner_cc),point(-lhw,-lhw)));
    field_lines.push_back(line(point( lhw,-inner_cc),point( lhw,-lhw)));
    field_lines.push_back(line(point(-lhw, inner_cc),point(-lhw, lhw)));
    field_lines.push_back(line(point( lhw, inner_cc),point( lhw, lhw)));

    // Center marker
    field_lines.push_back(line(point( spot_hl,-lhw),point( spot_hl, lhw)));
    field_lines.push_back(line(point(-spot_hl,-lhw),point(-spot_hl, lhw)));
    field_lines.push_back(line(point( spot_hl,-lhw),point( lhw,-lhw)));
    field_lines.push_back(line(point(-spot_hl,-lhw),point(-lhw,-lhw)));
    field_lines.push_back(line(point( spot_hl, lhw),point( lhw, lhw)));
    field_lines.push_back(line(point(-spot_hl, lhw),point(-lhw, lhw)));

    // outer
    field_lines.push_back(line(point( spot_front,-lhw),point( spot_front, lhw)));
    field_lines.push_back(line(point( spot_back,-lhw),point( spot_back, lhw)));
    field_lines.push_back(line(point(spot_center-lhw, spot_hl),point(spot_center+lhw, spot_hl)));
    field_lines.push_back(line(point(spot_center-lhw,-spot_hl),point(spot_center+lhw,-spot_hl)));
    // vertical inner
    field_lines.push_back(line(point(spot_center-lhw, lhw),point(spot_center-lhw, spot_hl)));
    field_lines.push_back(line(point(spot_center+lhw, lhw),point(spot_center+lhw, spot_hl)));
    field_lines.push_back(line(point(spot_center-lhw, -lhw),point(spot_center-lhw, -spot_hl)));
    field_lines.push_back(line(point(spot_center+lhw, -lhw),point(spot_center+lhw, -spot_hl)));
    // horizontal inner
    field_lines.push_back(line(point( spot_front, lhw),point(spot_center-lhw, lhw)));
    field_lines.push_back(line(point( spot_front,-lhw),point(spot_center-lhw,-lhw)));
    field_lines.push_back(line(point( spot_back, lhw),point(spot_center+lhw, lhw)));
    field_lines.push_back(line(point( spot_back,-lhw),point(spot_center+lhw,-lhw)));
    // outer
    field_lines.push_back(line(point(-spot_front,-lhw),point(-spot_front, lhw)));
    field_lines.push_back(line(point(-spot_back,-lhw),point(-spot_back, lhw)));
    field_lines.push_back(line(point(-(spot_center-lhw), spot_hl),point(-(spot_center+lhw), spot_hl)));
    field_lines.push_back(line(point(-(spot_center-lhw),-spot_hl),point(-(spot_center+lhw),-spot_hl)));
    // vertical inner
    field_lines.push_back(line(point(-(spot_center-lhw), lhw),point(-(spot_center-lhw), spot_hl)));
    field_lines.push_back(line(point(-(spot_center+lhw), lhw),point(-(spot_center+lhw), spot_hl)));
    field_lines.push_back(line(point(-(spot_center-lhw), -lhw),point(-(spot_center-lhw), -spot_hl)));
    field_lines.push_back(line(point(-(spot_center+lhw), -lhw),point(-(spot_center+lhw), -spot_hl)));
    // horizontal inner
    field_lines.push_back(line(point(-spot_front, lhw),point(-(spot_center-lhw), lhw)));
    field_lines.push_back(line(point(-spot_front,-lhw),point(-(spot_center-lhw),-lhw)));
    field_lines.push_back(line(point(-spot_back, lhw),point(-(spot_center+lhw), lhw)));
    field_lines.push_back(line(point(-spot_back,-lhw),point(-(spot_center+lhw),-lhw)));

    // Varibles for orientations.
    float camera_pitch = 0.698;
    float camera_yaw = 0.f;
    float camera_roll = 0.f;

    float body_pitch = 0.f;
    float body_yaw = 0.f;
    float body_roll = 0.f;

    // OpenGL coordinate system:
    // x is horizontal going from left to right
    // y is vertical from bottom to top
    // z is depth, pointing toward the camera.

    // Robot coordinate system:
    // x is depth going from the robot forward
    // y is vertical going from the right of the robot to the left.
    // z is vertical going from the foot of the robot toward the head.

    // OpenGL to Robot:
    // x -> y
    // z -> x
    // y -> -z
    // Note: the z axis is negative

    // Offsets for rotations to change from OpenGL coords to Robot coords.
    // These are hard-coded since they are the conversion from opengl to standard robot coordinated.
    const float pitch_offset = -90;
    const float yaw_offset = -90.f;
    const float roll_offset = 0.f;

    // Camera offset values. (These should not be hard-coded since they differ per robot type)
    const float camera_x_offset = 3.32f;
    const float camera_y_offset = 0.f;
    const float camera_z_offset = 3.44f;

    // Body offset values. (These should not be hard-coded since they differ per robot type)
    const float body_x_offset = 0.f;
    const float body_y_offset = 0.f;
    const float body_z_offset = 39.22f;

    // Get the sensor information.
    float temp = 0.f;   // temp variable for fetching sensor values.
    if(SensorData)  // Check if sensor data is available
    {
        // Camera pitch
        if(SensorData->getPosition(NUSensorsData::HeadPitch, temp))
        {
            camera_pitch += temp;
        }
        // Camera yaw
        if(SensorData->getPosition(NUSensorsData::HeadYaw, temp))
        {
            camera_yaw += temp;
        }
        // Camera roll - Our robots do not have thus at the moment.
        if(SensorData->getPosition(NUSensorsData::HeadRoll, temp))
        {
            camera_roll += temp;
        }
        // Camera Height
//        if(SensorData->getCameraHeight(temp))
//        {
//            std::cout << "Camera Height: " << temp << std::endl;
//        }
    }

    // Setup opengl drawing buffer.
    QGLPixelBuffer buffer(width, height);   // Initialise at correct resolution.
    buffer.makeCurrent();                   // Set buffer as target for OpenGL commands.

    // Initialise OpenGL for drawing
    glClearColor(0,0,0,0); // clear with no alpha for transparent background.
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Do clear
    glLineWidth(1.f);       // Drawing line width

    glMatrixMode(GL_PROJECTION);    // Set to projection to setup camera.
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST); // Use Best Perspective Calculations
    glLoadIdentity();   // Initialise matrix
    // Setup field of view
    gluPerspective(fov_vertical,(GLfloat)(fov_horizontal/fov_vertical),0.1f,10000.0f);

    glMatrixMode(GL_MODELVIEW); // Change back to model view matrix to render scene.

    // Move to correct orientation to match robot coordinate system.
    glRotatef(pitch_offset, 1.f, 0.f, 0.f);
    glRotatef(yaw_offset, 0.f, 0.f, 1.0f);
    glRotatef(roll_offset, 0.f, 1.f, 0.f);

    glTranslatef(camera_x_offset, camera_y_offset, -camera_z_offset);

    // Apply camera orientation
    glRotatef(mathGeneral::rad2deg(camera_pitch + calibration->camera_pitch_offset), 0.f, 1.f, 0.f);    //  Pitch
    glRotatef(mathGeneral::rad2deg(camera_yaw + calibration->camera_yaw_offset), 0.f, 0.f, -1.f);   //  Yaw
    glRotatef(mathGeneral::rad2deg(camera_roll + calibration->camera_roll_offset), 1.f, 0.f, 0.f);   //  Roll

    // Apply translation for body
    glTranslatef(body_x_offset, body_y_offset, -body_z_offset);

    // Apply Body Orientation
    glRotatef(mathGeneral::rad2deg(body_pitch+calibration->body_pitch_offset), 0.f, 1.f, 0.f);     // Pitch
    glRotatef(mathGeneral::rad2deg(body_yaw), 0.f, 0.f, -1.f);     // Yaw
    glRotatef(mathGeneral::rad2deg(body_roll+calibration->body_roll_offset), 1.f, 0.f, 0.f);   // Roll

    // Apply the robot heading
    glRotatef(mathGeneral::rad2deg(calibration->location_orientation), 0.f, 0.f, -1.f);
    // Position on field
    glTranslatef(calibration->location_x, calibration->location_y, 0.f); // Translation

    // Draw expected field.
    point start;
    point end;
    glColor3ub(255,255,255);    // Set draw colour (White)

    // Draw all of the lines.
    BOOST_FOREACH(line curr_line, field_lines)
    {
        glBegin(GL_LINES);                               // Start Lines
        start = curr_line.first;
        end = curr_line.second;
        glVertex3f(start.x, start.y, 0.f);             // Starting point
        glVertex3f(end.x, end.y, 0.f);             // Ending point
        glEnd();  // End Lines
    }

    // Draw the center circle.
    const float angle_offset = mathGeneral::deg2rad(90);
    const float arc_base_length = mathGeneral::deg2rad(180);
    DrawArc(0,0,inner_cc,angle_offset+inner_arc_start_angle,arc_base_length-2*inner_arc_start_angle, 100);
    DrawArc(0,0,inner_cc,angle_offset-inner_arc_start_angle,-arc_base_length+2*inner_arc_start_angle, 100);
    DrawArc(0,0,outer_cc,angle_offset+outer_arc_start_angle,arc_base_length-2*outer_arc_start_angle, 100);
    DrawArc(0,0,outer_cc,angle_offset-outer_arc_start_angle,-arc_base_length+2*outer_arc_start_angle, 100);

    // Get the drawn image from the buffer.
    buffer.doneCurrent();
    QImage image = buffer.toImage();

    // Save as a layer and emit the change.
    createDrawTextureImage(image, displayId);
    emit updatedDisplay(displayId, displays[displayId], width, height);
    return;
}
