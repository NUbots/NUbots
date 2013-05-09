#include <iostream>
#include <google/protobuf/stubs/common.h>

#include "NUbot.h"

#include "Tools/Threading/PeriodicSignalerThread.h"

#include "debug.h"
#include "nubotdataconfig.h"

using namespace std;

ofstream debug;
ofstream errorlog;

int main(int argc, const char *argv[]) 
{
	GOOGLE_PROTOBUF_VERIFY_VERSION;
	
    debug.open((DATA_DIR + "debug.log").c_str());
    errorlog.open((DATA_DIR + "error.log").c_str());
                  
    NUbot* nubot = new NUbot(argc, argv);
    PeriodicSignalerThread* helperthread = new PeriodicSignalerThread(string("DarwinSensorSignaler"), (ConditionalThread*) nubot->m_sensemove_thread, 10);
    helperthread->start();
    nubot->run();
    delete nubot;
	
	google::protobuf::ShutdownProtobufLibrary();
}
