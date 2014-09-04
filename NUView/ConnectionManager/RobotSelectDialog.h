/*! @file RobotSelectDialog.h
    @brief Declaration of NUView's RoboSelectDialog class to provide a little dialog to select a host.
     
    @class RobotSelectDialog
    @brief A class to display a little dialog to allow the selection of a host
     
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

#ifndef ROBOTSELECTDIALOG_H
#define ROBOTSELECTDIALOG_H

class BonjourProvider;
class NUHostInfo;

#include <QDialog>
#include <QHostInfo>
#include <string>
#include <list>


class QDialogButtonBox;
class QPushButton;
class QLabel;
class QTreeWidget;
class QTreeWidgetItem;

class RobotSelectDialog : public QDialog
{
Q_OBJECT
public:
    RobotSelectDialog(QWidget* parent = 0, BonjourProvider* provider = 0);
    ~RobotSelectDialog();
    
    std::vector<NUHostInfo>& getSelectedHosts();
    
private slots:
    void populateTree();
    void saveSelected();
private:
    void addService(const std::string& service, std::list<NUHostInfo>& hosts);
    
    void enableConnectButton();
private:
    BonjourProvider* m_bonjour;					//!< a pointer to the bonjour provider
    std::vector<NUHostInfo> m_selected_hosts;		//!< a list of the currently selected hosts
    
    QPushButton* m_connect_button;
    QPushButton* m_cancel_button;
    QDialogButtonBox* m_button_box;
    QTreeWidget* m_tree;
};

#endif // ROBOTSELECTDIALOG_H
