/*! @file BonjourServiceBrowser.cpp
    @brief Implementation of NUView's BonjourServiceBrowser class

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

#include "BonjourServiceBrowser.h"
#include "BonjourServiceResolver.h"
#include "NUHostInfo.h"

#include "debug.h"


/*! @brief Constructs a browser for a single service type
 	@param servicetype the type of service (eg. '_workstation._tcp')
 */
BonjourServiceBrowser::BonjourServiceBrowser(const std::string& servicetype)
{
    m_service_type = servicetype;
    initServiceBrowse();
	startServiceBrowse();
}

/*! @brief Destroys the browser */
BonjourServiceBrowser::~BonjourServiceBrowser()
{
    endServiceBrowse();
}

/*! @brief Returns the service type for this browser */
std::string& BonjourServiceBrowser::getServiceType()
{
    return m_service_type;
}

/*! @brief Returns the std::list of hosts that provide m_service_type */
std::list<NUHostInfo>& BonjourServiceBrowser::getHosts()
{
    return m_hosts;
}

/*! @brief Initialises the browser */
void BonjourServiceBrowser::initServiceBrowse()
{
    m_ref = 0;
    m_sockfd = -1;
    m_notifier = 0;
    pthread_mutex_init(&m_hosts_mutex, 0);
}

/*! @brief Starts the browsing service. Effectively, everytime a new service is found or one is lost onResolveCompleted is called.
 
           This is done using a long sequence of events. A QSocketNotifier is used to monitor activity on the socket used by the DNSServiceBrowse.
 		   When activity is observed, DNSServiceProcessResult is called, which in turn calls the callback onBrowseResults.
           onBrowseResults then uses a BonjourServiceResolver to resolve the information into a QHostInfo. When the BonjourServiceResolver is
           finished onResolveCompleted is called, and the resolver will now contain the ip address.
 */
void BonjourServiceBrowser::startServiceBrowse()
{
    DNSServiceErrorType err = DNSServiceBrowse(&m_ref, kDNSServiceFlagsBrowseDomains, 0, m_service_type.c_str(), 0, onBrowseResults, this);
    if (err != kDNSServiceErr_NoError)
        debug << "BonjourServiceBrowser::BonjourServiceBrowser. DNSServiceBrowse failed for " << m_service_type << ": " << err << std::endl;
    else
    {
        m_sockfd = DNSServiceRefSockFD(m_ref);
        if (m_sockfd != -1)
        {	// setup a notifier to call onSocketRead every time the socket is read
            m_notifier = new QSocketNotifier(m_sockfd, QSocketNotifier::Read, 0);
            connect(m_notifier, SIGNAL(activated(int)), this, SLOT(onSocketRead()));
        }
    }
}

/*! @brief Ends the browse service. Deallocates the DNSServiceRef, and deletes the QSocketNotifier
*/
void BonjourServiceBrowser::endServiceBrowse()
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

/*! @brief A slot for the activated signal of the QSocketNotifier monitoring the DNSServiceBrowse socket. */
void BonjourServiceBrowser::onSocketRead()
{
    m_notifier->setEnabled(false);
    DNSServiceProcessResult(m_ref);
    m_notifier->setEnabled(true);
}

/*! @brief The callback for the DNSServiceBrowse. This is the function where new services need to be added to the std::list. */
void BonjourServiceBrowser::onBrowseResults(DNSServiceRef, DNSServiceFlags flags, uint32_t iface, DNSServiceErrorType, const char* name, const char* type, const char* domain, void* context)
{
    BonjourServiceBrowser* browser = static_cast<BonjourServiceBrowser*>(context);
    BonjourServiceResolver* resolver = new BonjourServiceResolver(flags, iface, name, type, domain);
    connect(resolver, SIGNAL(resolveCompleted(BonjourServiceResolver*)), browser, SLOT(onResolveCompleted(BonjourServiceResolver*)));
}

/*! @brief A slot for the BonjourServiceResolver created in onBrowseResults resolveCompleted signal
 	@param resolver a pointer to the resolver itself. Use the pointer to getHostInfo(). Also delete the resolver when finished
 */
void BonjourServiceBrowser::onResolveCompleted(BonjourServiceResolver* resolver)
{
    if (resolver->isNewService())
        addHost(resolver->getHostInfo());
    else
        removeHost(resolver->getHostInfo());
    delete resolver;
    emit newBrowserInformation();
}

/*! @brief Adds a host to m_hosts */
void BonjourServiceBrowser::addHost(NUHostInfo& info)
{
    pthread_mutex_lock(&m_hosts_mutex);
    m_hosts.push_back(info);
    m_hosts.unique();
    pthread_mutex_unlock(&m_hosts_mutex);
}

/*! @brief Removes a host from m_hosts */
void BonjourServiceBrowser::removeHost(NUHostInfo& info)
{
    m_hosts.remove(info);
}

