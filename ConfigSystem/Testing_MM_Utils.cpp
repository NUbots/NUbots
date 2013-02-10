#include "Testing_MM_Utils.h"

#include <string>
#include <sstream>

using namespace ConfigSystem;

extern ConfigManager* config;


void initRandom()
{
    timeval t;
    gettimeofday(&t,NULL);
    seed = boost::mt19937( (int)t.tv_sec );
}

void printTestResult(std::string testName, bool result)
{
    std::cout << testName << ": "
              << (result? "pass." : "FAIL!") 
              << std::endl;
}

timespec diff(timespec start, timespec end)
{
    timespec temp;
    if ((end.tv_nsec-start.tv_nsec)<0) {
        temp.tv_sec = end.tv_sec-start.tv_sec-1;
        temp.tv_nsec = 1000000000 + end.tv_nsec-start.tv_nsec;
    } else {
        temp.tv_sec = end.tv_sec-start.tv_sec;
        temp.tv_nsec = end.tv_nsec-start.tv_nsec;
    }
    return temp;
}

timespec tt_startTime;
void startTimedTest()
{
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &tt_startTime);
}

void endTimedTest()
{
    timespec tt_endTime;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &tt_endTime);

    double tt = diff(tt_startTime, tt_endTime).tv_nsec / 1000000.0;
    double tt_Hz = 1.0 / (tt / 1000.0);

    // std::cout << "    time = " << tt << " ms." << std::endl;
    // std::cout << "    time = " << tt_Hz << " Hz." << std::endl;
    std::cout << "  time = " << tt << " ms (" << tt_Hz << " Hz)." << std::endl;
}


std::string makeRandomName(int len)
{
    // all the keyboard characcters
    // excepting: '.', 
    std::string lut = 
        // "0123456789"
        "abcdefghijklmnopqrstuvwxyz"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        // "`~!@#$%^&*()-_=+[{]}\\|;:'\",<>/?"
        ;

    boost::uniform_int<> distI(0, lut.length() - 1);
    boost::variate_generator<boost::mt19937&, boost::uniform_int<> > randInt(seed,distI);
    
    std::stringstream ss;
    
    for(int i = 0; i < len; i ++)
        ss << lut.at(randInt());

    return ss.str();
}
