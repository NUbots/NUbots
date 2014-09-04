/*! @file BonjourProvider.cpp
    @brief Implementation of NUView's BonjourProvider class

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

#include "BonjourProvider.h"
#include "BonjourServiceBrowser.h"
#include "NUHostInfo.h"

#include "debug.h"

/*! @brief Constructs a BonjourProvider over the specified services.
 	@param servicetypes the name of each of the services we want to monitor (eg. _workstation._tcp)
 */
BonjourProvider::BonjourProvider(const std::vector<std::string>& servicetypes)
{
    m_service_names = servicetypes;
    m_services.reserve(servicetypes.size());
    for (size_t i=0; i<servicetypes.size(); i++)
    {
        m_services.push_back(new BonjourServiceBrowser(servicetypes[i]));
        connect(m_services[i], SIGNAL(newBrowserInformation()), this, SLOT(onNewBrowserInformation()));
    }
}

/*! @brief Destructor for BonjourProvider */
BonjourProvider::~BonjourProvider()
{
    for (size_t i=0; i<m_services.size(); i++)
        delete m_services[i];
}

/* @brief Returns the names of the services we are monitoring */
std::vector<std::string>& BonjourProvider::getServices()
{
    return m_service_names;
}

/*! @brief Returns the lists of the hosts we have found
 			
 		   The hosts will be formatted as [serviceA, serviceB, .... , serviceN]
           where each serviceI is a list [host0, host1, ...., hostM]
 */
std::vector<std::list<NUHostInfo> > BonjourProvider::getHosts()
{
    std::vector<std::list<NUHostInfo> > hosts;
    hosts.reserve(m_services.size());
    for (size_t i=0; i<m_services.size(); i++)
        hosts.push_back(m_services[i]->getHosts());
    return hosts;
}

/*! @brief Looks up a hostname, and attempts to get the ip address for that host
 	@param name the hostname we want the ip address
 
 	@return a NUHostInfo containing both the full hostname, and its ip address. If the hostname can
 	        not be resolved an empty NUHostInfo will be returned
 */
NUHostInfo BonjourProvider::lookupHostName(std::string& name)
{
    std::vector<std::list<NUHostInfo> > hosts = getHosts();
    for (size_t i=0; i<hosts.size(); i++)
    {
        for (std::list<NUHostInfo>::iterator it = hosts[i].begin(); it != hosts[i].end(); ++it)
        {
            if ((*it) == name)
                return *it;
        }
    }
    return NUHostInfo();
}

/*! @brief A signal to indicate that a new host has been found. The idea is to connect this with an update of the gui */
void BonjourProvider::onNewBrowserInformation()
{
    emit newBrowserInformation();
}

