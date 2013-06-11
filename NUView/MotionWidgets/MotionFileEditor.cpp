/*! @file MotionFileEditor.cpp
    @brief Implementation of NUView's MotionFileEditor class for manual editing of motion files

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

#include "MotionFileEditor.h"
#include "MotionFileSyntaxHighlighter.h"

#include <iostream>
#include <sstream>

#include "debug.h"
#include "nubotdataconfig.h"			// need this to set the default position for the filedialog

MotionFileEditor::MotionFileEditor(const std::string& filepath, QWidget *parent): QWidget(parent)
{
    m_filepath = filepath;
    
    QFileInfo fileinfo(m_filepath.c_str());			// Use QFileInfo to split the filename from the path so I can use it as the title
    setObjectName(fileinfo.fileName());
    setWindowTitle(fileinfo.fileName());
    setAttribute(Qt::WA_DeleteOnClose, true);		// This is a neat trick for the widget to delete itself when its closed
    
    m_layout = new QVBoxLayout();
	
    m_title = new QLabel(m_filepath.c_str());
    m_layout->addWidget(m_title);
    
    m_editor = new QTextEdit();
    m_editor->setLineWrapMode(QTextEdit::NoWrap);
    m_highlighter = new MotionFileSyntaxHighlighter(m_editor->document());
    m_layout->addWidget(m_editor);
    
    m_button_layout = new QHBoxLayout();
    m_send_button = new QPushButton("Send");
    m_button_layout->addWidget(m_send_button);
    m_save_button = new QPushButton("Save");
    m_button_layout->addWidget(m_save_button);
    m_revert_button = new QPushButton("Revert");
    m_button_layout->addWidget(m_revert_button);
    m_layout->addLayout(m_button_layout);
    
    setLayout(m_layout);
    
    connect(m_send_button, SIGNAL(released()), this, SLOT(send()));
    connect(m_save_button, SIGNAL(released()), this, SLOT(save()));
    connect(m_revert_button, SIGNAL(released()), this, SLOT(load()));
    
    load();
    setFocus();
}

MotionFileEditor::~MotionFileEditor()
{
    delete m_editor;
    delete m_send_button;
    delete m_save_button;
    delete m_revert_button;
    delete m_button_layout;
    delete m_layout;
}

void MotionFileEditor::save()
{
    ofstream file(m_filepath.c_str());
    if (file.is_open())
    {
        std::string buffer = m_editor->toPlainText().toStdString();
        file.write(buffer.c_str(), buffer.size());
        file.close();
    }
}

void MotionFileEditor::send()
{
    emit sendRequested(m_editor->toPlainText().toStdString());
}

void MotionFileEditor::load()
{
    ifstream file(m_filepath.c_str(), ios::in|ios::binary|ios::ate);
    if (file.is_open())
    {	// if the file opens successfully, just do a simple binary read of the entire contents
        int size = (int)file.tellg();
        char* buffer = new char[size+1];
        buffer[size] = '\0';				// make it a nice cstring by adding a null termination character
		file.seekg(0, ios::beg);
        file.read(buffer, size);
        file.close();
        m_editor->setText(buffer);		// set the contents of the editor to be the contents of the buffer
    }
}

void MotionFileEditor::checkSyntax()
{
    std::stringstream text(m_editor->toPlainText().toStdString());
    std::string line;
    while (text.good())
    {
    	getline(text, line);
        if (not bracketsOK(line))
            debug << "syntax broken" << std::endl;
    }
    // for each line check that there are an equal number of ([]) in each line
    
}

bool MotionFileEditor::bracketsOK(const std::string& line)
{
    int brackets = 0;
    for (size_t i=0; i<line.size(); i++)
    {
        if (line[i] == '[' or line[i] == '(' or line[i] == '{')
            brackets++;
        else if (line[i] == ']' or line[i] == ')' or line[i] == '}')
            brackets--;
    }
    return brackets == 0;
}

