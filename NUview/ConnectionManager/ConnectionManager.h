/*! @file ConnectionManager.h
    @brief Declaration of NUView's ConnectionManager class for managing connections with NUbot's
 
    @class ConnectionManager
    @brief A widget for managing connections with NUbot's
 
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

#ifndef CONNECTIONMANAGER_H
#define CONNECTIONMANAGER_H

class BonjourProvider;
#include "NUHostInfo.h"

#include <QtGui>
#include <vector>
using namespace std;

class ConnectionManager : public QWidget
{
    Q_OBJECT
public:
    ConnectionManager(QWidget* parent);
    ~ConnectionManager();

signals:
    void newHosts(vector<NUHostInfo> hosts);
private slots:
    void onListButton();
    void onInputFinished();
private:
    QHBoxLayout* m_layout;
    QPushButton* m_list_button;
    QLineEdit* m_user_ip_input;
    
    BonjourProvider* m_bonjour;
    vector<NUHostInfo> m_current_hosts;
};

#endif

