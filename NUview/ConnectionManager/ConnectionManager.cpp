/*! @file ConnectionManager.cpp
    @brief Implementation of NUView's ConnectionManager class

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

#include "ConnectionManager.h"

#include "robotSelectDialog.h"
#include "bonjourserviceresolver.h"


ConnectionManager::ConnectionManager(QWidget* parent) : QWidget(parent)
{
    setObjectName("Connection Manager");
    
    m_list_button = new QPushButton("List NUbots");
    connect(m_list_button, SIGNAL(released()), this, SLOT(onListButton()));
    
    m_user_ip_input = new QLineEdit("255.255.255.255");
    connect(m_user_ip_input, SIGNAL(returnPressed()), this, SLOT(onInputFinished()));
    
    m_status_display = new QLabel();
    drawStatus(Qt::green);
    
    m_vision_checkbox = new QCheckBox("Vision");
    connect(m_vision_checkbox, SIGNAL(stateChanged()), this, SLOT(onVisionChecked()));
    m_localisation_checkbox = new QCheckBox("Localisation");
    connect(m_localisation_checkbox, SIGNAL(stateChanged()), this, SLOT(onLocalisationChecked()));
    m_sensors_checkbox = new QCheckBox("Sensors");
    connect(m_sensors_checkbox, SIGNAL(stateChanged()), this, SLOT(onSensorsChecked()));
    
    m_layout = new QHBoxLayout();
    m_layout->addWidget(m_list_button);
    m_layout->addWidget(m_user_ip_input);
    m_layout->addWidget(m_status_display);
    m_layout->addWidget(m_vision_checkbox);
    m_layout->addWidget(m_localisation_checkbox);
    m_layout->addWidget(m_sensors_checkbox);
    
    setLayout(m_layout);
}

ConnectionManager::~ConnectionManager()
{
    delete m_list_button;
    m_list_button = 0;
    delete m_user_ip_input;
    m_user_ip_input = 0;
    delete m_status_display;
    m_status_display = 0;
    delete m_vision_checkbox;
    m_vision_checkbox = 0;
    delete m_localisation_checkbox;
    m_localisation_checkbox = 0;
    delete m_sensors_checkbox;
    m_sensors_checkbox = 0;
    
    delete m_layout;
    m_layout = 0;
}

/*! @brief Draws a small circle in m_status_display 
 	@param colour the fill colour of the circle
 */
void ConnectionManager::drawStatus(const QColor& colour)
{
	QPixmap temp(21,21);
    temp.fill(Qt::transparent);
    QPainter p(&temp);
    p.setRenderHint(QPainter::Antialiasing, true);
    QBrush brush(colour);
    p.setBrush(brush);
    p.drawEllipse(3,3,15,15);

    m_status_display->setPixmap(temp);
}

/*! @brief Slot for the released() signal of m_list_button
 */
void ConnectionManager::onListButton()
{
    robotSelectDialog test(this, "_nuview._tcp");
    if(test.exec())
    {
        BonjourRecord bonjourHost = test.getBonjourHost();
        if (!bonjourResolver)
        {
            bonjourResolver = new BonjourServiceResolver(this);
            //connect(bonjourResolver, SIGNAL(bonjourRecordResolved(const QHostInfo &, int)), this, SLOT(PrintConnectionInfo(const QHostInfo &, int)));
        }
        bonjourResolver->resolveBonjourRecord(bonjourHost);
    }
}

/*! @brief Slot for the returnPressed() signal of m_user_ip_input
 */
void ConnectionManager::onInputFinished()
{
}

/*! @brief Slot for the stateChanged() signal of m_vision_checkbox
 */
void ConnectionManager::onVisionChecked()
{
}

/*! @brief Slot for the stateChanged() signal of m_localisation_checkbox
 */
void ConnectionManager::onLocalisationChecked()
{
}

/*! @brief Slot for the stateChanged() signal of m_sensors_checkbox
 */
void ConnectionManager::onSensorsChecked()
{
}

