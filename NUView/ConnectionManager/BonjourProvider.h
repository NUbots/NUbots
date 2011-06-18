/*! @file BonjourProvider.h
    @brief Declaration of NUView's BonjourProvider class for managing a list of avaliable services
 		   and resolving names/services into more useful ip addresses.
 
    @class BonjourProvider
    @brief A class for managing a list of available services and resolving name/services into more useful
           id addresses.
 
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

#ifndef BonjourProvider_H
#define BonjourProvider_H

class BonjourServiceBrowser;
class NUHostInfo;

#include <QtGui>

#include <dns_sd.h>
#include <vector>
#include <string>
using namespace std;

class BonjourProvider : public QObject
{
    Q_OBJECT
public:
    BonjourProvider(const vector<string>& servicetypes);
    ~BonjourProvider(); 
    
    vector<string>& getServices();
    vector<list<NUHostInfo> > getHosts();
    
    NUHostInfo lookupHostName(string& name);
signals:
    void newBrowserInformation();
private slots:
    void onNewBrowserInformation();
private:
    vector<string> m_service_names;					//!< the names of the bonjour services we are interested in
    vector<BonjourServiceBrowser*> m_services;		//!< a browser over each of the services in m_service_names
};

#endif

