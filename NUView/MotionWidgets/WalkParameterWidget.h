/*! @file WalkParameterWidget.h
 	@brief Declaration of NUView's WalkParameterWidget class for manual tuning of walk parameters
 
 	@class WalkParameterWidget
 	@brief A widget for manual tuning of WalkParameterWidget
 
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


#ifndef WALKPARAMETERWIDGET_H
#define WALKPARAMETERWIDGET_H

#include "../ConnectionManager/NUHostInfo.h"
class MotionFileEditor;

#include <QtGui>
#include <string>


class WalkParameterWidget : public QWidget
{
    Q_OBJECT
public:
    WalkParameterWidget(QWidget *parent = 0);
    ~WalkParameterWidget();
    
private slots:
    void onNewHost(std::vector<NUHostInfo> hosts);
    
    void onSelectButtonPressed();
    void onSendRequested(std::string text);
    
private:
    QHBoxLayout* m_layout;
    QCheckBox* m_enabled_box;
    QPushButton* m_select_file_button;
    
    MotionFileEditor* m_editor;
};

#endif
