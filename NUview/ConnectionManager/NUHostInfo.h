/*! @file NUHostInfo.h
    @brief Declaration of NUView's NUHostInfo class to provide a standard way of storing hostnames and ip addresses
 
    @class NUHostInfo
 	@brief A class to provide a standard way of storing hostnames and ip addresses. 
           It is a QMetaType so that it can be easily used inside Qt gui items.
 
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

#ifndef NUHostInfo_H
#define NUHostInfo_H

#include <QtNetwork/QHostInfo>
#include <QtCore/QMetaType>

#include <string>
#include <ostream>

using namespace std;

class NUHostInfo
{
public:
    NUHostInfo() {};
    NUHostInfo(const QHostInfo& qinfo)
    {
        m_hostname = qinfo.hostName().toStdString();
        if (not qinfo.addresses().isEmpty())
            m_address = qinfo.addresses().last().toString().toStdString();
    }
    NUHostInfo(const string& hostname, const string& address)
    {
        m_hostname = hostname;
        m_address = address;
    }
    
    string& getHostName() {return m_hostname;};
    string& getAddress() {return m_address;};
    bool empty() {return m_hostname.empty() and m_address.empty();};
    
    bool operator==(const NUHostInfo& other)
    {
        return (m_hostname.compare(other.m_hostname) == 0) and (m_address.compare(other.m_address) == 0);
    }
    
    bool operator==(const string& name)
    {
        if (not name.empty())
        {
            if (isdigit(name[0]))
                return m_address.compare(name) == 0;
            else
            {
                string hostname(name);
                if (hostname.find(".local") == string::npos)
                    hostname += ".local.";
                if (hostname[hostname.size()-1] != '.')
                    hostname += '.';
                
                return m_hostname.compare(hostname) == 0;
            }
        }
        else
        	return empty();
    }
    
    bool operator<(const NUHostInfo& other)
    {
        return m_hostname.compare(other.m_hostname) < 0;
    }
    
    friend ostream& operator<<(ostream& o, const NUHostInfo& i)
    {
        o << "[" << i.m_hostname << ", " << i.m_address << "]";
        return o;
    }
private:
    string m_hostname;
    string m_address;
};

Q_DECLARE_METATYPE(NUHostInfo)

#endif

