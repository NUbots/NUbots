/*! @file WalkParameterWidget.cpp
    @brief Implementation of NUView's WalkParameterWidget class for manual tuning of walk parameters

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

#include "WalkParameterWidget.h"
#ifndef WIN32
    #include "ConnectionManager/ConnectionManager.h"
#endif
#include "MotionFileEditor.h"

#include "../NUViewIO/NUViewIO.h"
#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/Jobs/MotionJobs/WalkParametersJob.h"

#include <sstream>

#include "debug.h"
#include "nubotdataconfig.h"			// need this to set the default position for the filedialog

WalkParameterWidget::WalkParameterWidget(QWidget *parent): QWidget(parent)
{
    setObjectName(tr("Walk"));
    setWindowTitle(tr("Walk"));
    #ifndef WIN32
        connect(ConnectionMan, SIGNAL(newHosts(std::vector<NUHostInfo>)), this, SLOT(onNewHost(std::vector<NUHostInfo>)));
    #endif
    //m_job_list = new JobList();
    
    m_select_file_button = new QPushButton("Open");
    connect(m_select_file_button, SIGNAL(released()), this, SLOT(onSelectButtonPressed()));
    
    m_enabled_box = new QCheckBox("Enabled");
    
    m_layout = new QHBoxLayout();
    m_layout->addWidget(m_select_file_button);
    m_layout->addWidget(m_enabled_box);
    setLayout(m_layout);
    
    m_editor = 0;
}


WalkParameterWidget::~WalkParameterWidget()
{
    //delete m_job_list;
}


void WalkParameterWidget::onNewHost(std::vector<NUHostInfo> hosts)
{
    if (not hosts.empty())
        nuio->setJobAddress(hosts.front().getAddress());
}

void WalkParameterWidget::onSelectButtonPressed()
{
    QFileDialog dialog(this, "Select the Walk Parameter File", DATA_DIR.c_str(), "*.cfg;*.num");
    int result = dialog.exec();
    
    QStringList files = dialog.selectedFiles();
    if (result == QDialog::Accepted and not files.empty())
    {
        m_editor = new MotionFileEditor(files.front().toStdString());
        connect(m_editor, SIGNAL(sendRequested(std::string)), this, SLOT(onSendRequested(std::string)));
        m_editor->show();
    }
}

void WalkParameterWidget::onSendRequested(std::string text)
{
    debug << "sending" << std::endl;
    JobList jobs;
    WalkParameters parameters;
    
    std::stringstream ss(text);
    ss >> parameters;
    jobs.addMotionJob(new WalkParametersJob(parameters));
    *nuio << jobs;
    
    debug << parameters;
}

