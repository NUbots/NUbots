/*! @file ALNAO.h
 @brief Implementation of ALNAO class.
 
 @author Jason Kulk
 
 Copyright (c) 2009 Jason Kulk
 
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

#include "NUNAO.h"
#include "NUbot.h"
#include "NUbot/SenseMoveThread.h"
#include <dcmproxy.h>
using namespace AL;

ofstream debug;
ofstream errorlog;
ALPtr<ALBroker> NUNAO::m_broker;

NUNAO::NUNAO(ALPtr<ALBroker> pBroker, const string& pName): ALModule(pBroker, pName)
{
    debug << "NUNAO.cpp: NUNAO::NUNAO" << endl;
    m_broker = pBroker;
    m_nubot = new NUbot(0, NULL);
    getParentBroker()->getProxy("DCM")->getModule()->atPostProcess(boost::bind<void>(&SenseMoveThread::startLoop, m_nubot->m_sensemove_thread));
    m_nubot->run();
}

NUNAO::~NUNAO()
{
    delete m_nubot;
}

extern "C" int _createModule(ALPtr<ALBroker> pBroker)
{
    debug.open("/var/log/debug.log");
    //debug.open("/home/nao/log/debug.log");
    debug << "NUbot Debug Log" << endl;
    debug << "NUNAO.cpp: _createModule" << endl;
    errorlog.open("/var/log/error.log");
    //errorlog.open("/home/nao/log/error.log");
    errorlog << "NUbot Error Log" << endl;
    ALModule::createModule<NUNAO>(pBroker, "NUNAO");
    return 0;
}

extern "C" int _closeModule()
{
    return 0;
}





