/*! @file RobotSelectDialog.h
 	@brief Declaration of NUView's RoboSelectDialog class to provide a little dialog to select a host.
 
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

#include "RobotSelectDialog.h"

#include "BonjourProvider.h"
#include "NUHostInfo.h"

#include <QtGui>

#include "debug.h"
#include "Tools/Math/StlVector.h"

/*! @brief Creates a RobotSelectDialog 
 	@param parent the parent Qt widget
 	@param provider the bonjour provider, this is used to get hosts from bonjour
 */
RobotSelectDialog::RobotSelectDialog(QWidget * parent, BonjourProvider* provider): QDialog(parent)
{
    m_bonjour = provider;
    connect(m_bonjour, SIGNAL(newBrowserInformation()), this, SLOT(populateTree()));		// connect the populate tree function to the new browser information signal 
    																						// (ie the populate tree function will be called each time a new host is found)
    m_tree = new QTreeWidget(this);

    QStringList labels;
    labels.append("Hosts");
    labels.append("Addresses");
    m_tree->setHeaderLabels(labels);
    m_tree->setSelectionMode(QAbstractItemView::ExtendedSelection);

    m_connect_button = new QPushButton(tr("Connect"));
    m_connect_button->setDefault(true);
    m_connect_button->setEnabled(false);

    m_cancel_button = new QPushButton(tr("Cancel"));

    m_button_box = new QDialogButtonBox;
    m_button_box->addButton(m_connect_button, QDialogButtonBox::AcceptRole);
    m_button_box->addButton(m_cancel_button, QDialogButtonBox::RejectRole);
    connect(m_button_box, SIGNAL(accepted()), this, SLOT(saveSelected()));
    connect(m_button_box, SIGNAL(accepted()), this, SLOT(accept()));
    connect(m_button_box, SIGNAL(rejected()), this, SLOT(reject()));

    QGridLayout *mainLayout = new QGridLayout;
    mainLayout->addWidget(m_tree, 0, 0, 2, 2);
    mainLayout->addWidget(m_button_box, 3, 0, 1, 2);
    setLayout(mainLayout);

    setWindowTitle(tr("Robot Selection"));
    m_tree->setFocus();
    
    populateTree();
}

/*! @brief Destroy the RobotSelectDialog */
RobotSelectDialog::~RobotSelectDialog()
{
    delete m_connect_button;
    delete m_cancel_button;
    delete m_button_box;
    delete m_tree;				// Note that this will also delete each item in the tree
}

/*! @brief Returns all of the currently selected hosts */
std::vector<NUHostInfo>& RobotSelectDialog::getSelectedHosts()
{
    return m_selected_hosts;
}

/*! @brief Populates m_tree with all of the hosts stored in each of m_bonjour's services */
void RobotSelectDialog::populateTree()
{
    m_tree->clear();
    
    std::vector<std::string> services = m_bonjour->getServices();
    std::vector<std::list<NUHostInfo> > hosts = m_bonjour->getHosts();
    for (size_t i=0; i<services.size(); i++)
    	addService(services[i], hosts[i]);
    
    m_tree->resizeColumnToContents(0);
    enableConnectButton();
}

/*! @brief Populates a node with all of the hosts that provide a single service 
 	@param service the name of the service
 	@param hosts the list of hosts
 */
void RobotSelectDialog::addService(const std::string& service, std::list<NUHostInfo>& hosts)
{
    QTreeWidgetItem* node = new QTreeWidgetItem(m_tree, QStringList() << service.c_str());
    for (std::list<NUHostInfo>::iterator it = hosts.begin(); it != hosts.end(); ++it)
    {
        NUHostInfo info = *it;
        QStringList text;			// a list of strings to display in the gui (name, address)
        QVariant data;				// a data variable to associate with each item (the NUHostInfo)
        
        text.append(info.getHostName().c_str());
        text.append(info.getAddress().c_str());
    	data.setValue(info);
        
        QTreeWidgetItem* child = new QTreeWidgetItem(node, text);
        child->setData(0, Qt::UserRole, data);
    }
    node->setExpanded(true);
}

/*! @brief Saves the selected hosts in the tree to m_selected_hosts. If no hosts are selected then m_selected_hosts will be empty. */
void RobotSelectDialog::saveSelected()
{
    m_selected_hosts.clear();
    
    QList<QTreeWidgetItem*> selected_items = m_tree->selectedItems();
    for (int i=0; i<selected_items.size(); i++)
    	m_selected_hosts.push_back(selected_items.at(i)->data(0,Qt::UserRole).value<NUHostInfo>());
}

/*! @brief Enables the connect button */
void RobotSelectDialog::enableConnectButton()
{
    m_connect_button->setEnabled(m_tree->invisibleRootItem()->childCount() != 0);
}
