/*! @file BonjourServiceBrowser.h
    @brief Declaration of NUView's BonjourServiceBrowser class for providing an interface
		   to a list of servers providing a single bonjour service. 
 
    @class BonjourProvider
 	@brief A class for providing an interface to a list of servers providing a single bonjour service.
 
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

#ifndef BonjourServiceBrowser_H
#define BonjourServiceBrowser_H

class BonjourServiceResolver;
class NUHostInfo;

#include <QtGui>
#include <dns_sd.h>
#include <pthread.h>
#include <string>
using namespace std;

class BonjourServiceBrowser : public QObject
{
    Q_OBJECT
public:
    BonjourServiceBrowser(const string& servicetype);
    ~BonjourServiceBrowser();
    
    string& getServiceType();
    list<NUHostInfo>& getHosts();
signals:
    void newBrowserInformation();
private:
    void initServiceBrowse();
    void startServiceBrowse();
    void endServiceBrowse();
    
    static void onBrowseResults(DNSServiceRef sdref, DNSServiceFlags flags, uint32_t iface, DNSServiceErrorType err, const char* name, const char* type, const char* domain, void* context);
    void addHost(NUHostInfo& info);
    void removeHost(NUHostInfo& info);
private slots:
	void onSocketRead();   
    void onResolveCompleted(BonjourServiceResolver* resolver);
private:
    string m_service_type;				//!< The name of the service (eg. '_workstation._tcp')
    DNSServiceRef m_ref;				//!< A service ref to use for browsing
    int m_sockfd;						//!< The file descriptor for the socket used by m_browse_service
    QSocketNotifier* m_notifier;		//!< A socket notifier for monitoring m_browse_sockfd
    
    pthread_mutex_t m_hosts_mutex;		//!< We need to protect the hosts list with a mutex
    list<NUHostInfo> m_hosts;			//!< A list of all of the servers that provide this service 
};

#endif

