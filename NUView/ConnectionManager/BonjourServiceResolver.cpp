/*! @file BonjourServiceResolver.cpp
    @brief Implementation of NUView's BonjourServiceResolver class

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

#include "BonjourServiceResolver.h"

#include "debug.h"


/*! @brief Constructs a provider for a single service
 	@param iface the interface number
 	@param name the hostname
 	@param type the service type
 	@param domain the domain
 */
BonjourServiceResolver::BonjourServiceResolver(DNSServiceFlags flags, uint32_t iface, const char* name, const char* type, const char* domain)
{
    initServiceResolve();
    m_service_add = flags & kDNSServiceFlagsAdd;
    startServiceResolve(iface, name, type, domain);
}

/*! @brief Destroys the resolver */
BonjourServiceResolver::~BonjourServiceResolver()
{
    endServiceResolve();
}

/*! @brief Returns true if the service is new, false is the service has disappeared. */
bool BonjourServiceResolver::isNewService()
{
    return m_service_add;
}

/*! @brief Returns the QHostInfo for the bonjour service we have resolved. */
NUHostInfo& BonjourServiceResolver::getHostInfo()
{
    return m_info;
}

/*! @brief Initialises the service resolver */
void BonjourServiceResolver::initServiceResolve()
{
    // each of these needs to be created each time a new service is found
    m_ref = 0;
    m_sockfd = -1;
    m_notifier = 0;
}

/*! @brief Starts the resolving service. Effectively, onResolveResults is called sometime later when the resolve is completed.
 
 		   This is done using a QSocketNotifer to monitor activity on the socket used by the DNSServiceResolve.
           When activity is observe DNSServiceProcessResult is called, which in turn calls the onResolveResults
 	
 	@param iface
 	@param name
 	@param type
 	@param domain
 */
void BonjourServiceResolver::startServiceResolve(uint32_t iface, const char* name, const char* type, const char* domain)
{
    DNSServiceErrorType err = DNSServiceResolve(&m_ref, 0, iface, name, type, domain, onResolveResults, this);
    if (err != kDNSServiceErr_NoError)
        debug << "BonjourServiceResolver::startServiceResolve. DNSServiceResolve failed for " << name << " " << type << endl;
    else
    {
        m_sockfd = DNSServiceRefSockFD(m_ref);
        if (m_sockfd != -1)
        {	// setup a notifier to call onResolveSocketRead every time the socket is read
            m_notifier = new QSocketNotifier(m_sockfd, QSocketNotifier::Read, this);
            connect(m_notifier, SIGNAL(activated(int)), this, SLOT(onSocketRead()));
        }
    }
}

/*! @brief Ends the resolve service */
void BonjourServiceResolver::endServiceResolve()
{
    if (m_ref)
    {
        DNSServiceRefDeallocate(m_ref);
        m_ref = 0;
        m_sockfd = -1;
        delete m_notifier;
        m_notifier = 0;
    }
}

/*! @brief A slot for the activated signal of the QSocketNotifier monitoring the DNSServiceResolve */
void BonjourServiceResolver::onSocketRead()
{
    m_notifier->setEnabled(false);             
    DNSServiceProcessResult(m_ref);
}

void BonjourServiceResolver::onLookupCompleted(const QHostInfo& info)
{
    if (info.error() == QHostInfo::NoError) 
        m_info = NUHostInfo(info);
    emit resolveCompleted(this);
}
void BonjourServiceResolver::onResolveResults(DNSServiceRef, DNSServiceFlags, uint32_t, DNSServiceErrorType, const char*, const char* hostname, uint16_t, uint16_t, const unsigned char*, void* context)
{
    string temp(hostname);				// There is a backwards compatibility problem:
    size_t length = temp.size();		// hostnames used to have a '.' at the end eg. smacbook.local.
    if (temp[length-1] == '.')			// However, on my laptop such a hostname is invalid
        temp.erase(length-1);			// Thus, if there is a '.' on the end remove it
    QHostInfo::lookupHost(temp.c_str(), (QObject*) context, SLOT(onLookupCompleted(QHostInfo)));
}


