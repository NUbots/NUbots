/*! @file MotionFileSyntaxHighlighter.h
 	@brief Declaration of NUView's MotionFileSyntaxHighlighter class for highlighting syntax of motion files (walk parameters and scripts)
 
 	@class MotionFileSyntaxHighlighter
 	@brief A widget for highlighting syntax of motion files (walk parameters and scripts)
 
 @author Jason Kulk
 
     Copyright (c) 2011 Jason Kulk
     
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


#ifndef MotionFileSyntaxHighlighter_H
#define MotionFileSyntaxHighlighter_H

#include <QtGui>
#include <string>
#include <vector>
using namespace std;

class MotionFileSyntaxHighlighter : public QSyntaxHighlighter
{
    Q_OBJECT
public:
    MotionFileSyntaxHighlighter(QTextDocument *parent = 0);
    ~MotionFileSyntaxHighlighter();
    
private:
    void highlightBlock(const QString& text);
    void checkSyntax();
    bool bracketsOK(const string& line);
};

#endif
