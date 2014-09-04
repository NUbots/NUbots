/*! @file BonjourServiceResolver.h
    @brief Declaration of NUView's BonjourServiceResolver class for providing an interface
		   to resolving a single bonjour service into a useful ip address
 
    @class BonjourServiceResolver
 	@brief A class for providing an interface to resolving a single bonjour service into a useful ip address.
 
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

#ifndef BonjourServiceResolver_H
#define BonjourServiceResolver_H

#include "NUHostInfo.h"
#include <QtGui>
#include <QtNetwork/QHostInfo>

#include <dns_sd.h>
#include <string>
#include <list>


class BonjourServiceResolver : public QObject
{
    Q_OBJECT
public:
    BonjourServiceResolver(DNSServiceFlags flags, uint32_t iface, const char* name, const char* type, const char* domain);
    ~BonjourServiceResolver();
    
    bool isNewService();
    NUHostInfo& getHostInfo();
signals:
    void resolveCompleted(BonjourServiceResolver* resolver);
private:
    void initServiceResolve();
    void startServiceResolve(uint32_t iface, const char* name, const char* type, const char* domain);
    void endServiceResolve();

    static void onResolveResults(DNSServiceRef sdref, DNSServiceFlags flags, uint32_t iface, DNSServiceErrorType err, const char* domain, const char* hostname, uint16_t port, uint16_t mesglength, const unsigned char* mesg, void* context);
private slots:   
    void onSocketRead();
    void onLookupCompleted(const QHostInfo& info);
private:
    DNSServiceRef m_ref;					//!< A service ref to use for resolving
    int m_sockfd;							//!< The file descriptor for the socket used by m_resolve_service
    QSocketNotifier* m_notifier;			//!< A socket notifier for monitoring m_resolve_sockfd
    
    bool m_service_add;
    NUHostInfo m_info;
};

#endif

