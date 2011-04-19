/*! @file MotionFileEditor.h
 	@brief Declaration of NUView's MotionFileEditor class for manually editing motion files (walk parameters and scripts)
 
 	@class MotionFileEditor
 	@brief A widget for editing motion files (walk parameters and scripts)
 
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


#ifndef MotionFileEditor_H
#define MotionFileEditor_H

#include <QtGui>
#include <string>
using namespace std;

class MotionFileEditor : public QWidget
{
    Q_OBJECT
public:
    MotionFileEditor(const string& filepath, QWidget *parent = 0);
    ~MotionFileEditor();
    
signals:
    void sendRequested(string text);
    
private slots:
    void send();
    void save();
    void load();
    
private:
    string m_filepath;
    
    QVBoxLayout* m_layout;
    QHBoxLayout* m_button_layout;
    
    QLabel* m_title;
    QTextEdit* m_editor;
    QPushButton* m_send_button;
    QPushButton* m_save_button;
    QPushButton* m_revert_button;
};

#endif
