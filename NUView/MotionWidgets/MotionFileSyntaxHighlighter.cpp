/*! @file MotionFileSyntaxHighlighter.cpp
    @brief Implementation of NUView's MotionFileSyntaxHighlighter class for highlighting syntax of motion files

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

#include "MotionFileSyntaxHighlighter.h"

#include "Tools/Math/StlVector.h"
#include "debug.h"

MotionFileSyntaxHighlighter::MotionFileSyntaxHighlighter(QTextDocument *parent): QSyntaxHighlighter(parent)
{
}

MotionFileSyntaxHighlighter::~MotionFileSyntaxHighlighter()
{
}


void MotionFileSyntaxHighlighter::highlightBlock(const QString& text)
{
    if (not bracketsOK(text.toStdString()))
        setFormat(0, text.length(), Qt::red);
    else
    {
        for (int i = 0; i < text.length(); ++i) 
        {
            if (!text.at(i).isLetterOrNumber())
                setFormat(i, 1, Qt::green);
        }
    }
}

void MotionFileSyntaxHighlighter::checkSyntax()
{
    /*std::stringstream text(m_editor->toPlainText().toStdString());
    std::string line;
    while (text.good())
    {
    	getline(text, line);
        if (not bracketsOK(line))
            debug << "syntax broken" << std::endl;
    }*/
    // for each line check that there are an equal number of ([]) in each line
    
}

bool MotionFileSyntaxHighlighter::bracketsOK(const std::string& line)
{
    int brackets = 0;				// the sum of the brackets +1 for [, -1 for ]
    for (size_t i=0; i<line.size(); i++)
    {
        if (line[i] == '[' or line[i] == '(' or line[i] == '{')
            brackets++;
        else if (line[i] == ']' or line[i] == ')' or line[i] == '}')
            brackets--;
    }
    
    return brackets == 0;
}

