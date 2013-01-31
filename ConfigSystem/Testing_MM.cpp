/*! 
    @file TestConfig.cpp
    @brief Tests for the Config System.
  
    @author Mitchell Metcalfe
 
  Copyright (c) 2012 Mitchell Metcalfe
    
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

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <string>
#include <vector>
#include <exception>
#include <boost/random/uniform_int.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>

#include <sys/time.h>

#include "ConfigManager.h"
#include "ConfigRange.h"
//#include "ConfigStorageManager.h"

#include "Module.h"

using namespace ConfigSystem;

ConfigManager* config;

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

int main(void)
{
    timeval t;
    gettimeofday(&t,NULL);
    boost::mt19937 seed( (int)t.tv_sec );

    boost::uniform_real<> distR(0, 1);
    boost::variate_generator<boost::mt19937&, boost::uniform_real<> > randReal(seed,distR);
    
    boost::uniform_int<> distI(0, 255);
    boost::variate_generator<boost::mt19937&, boost::uniform_int<> > randInt(seed,distI);

    // // Repeat all tests this many times
    // for(int num_test_repeats = 100; num_test_repeats > 0; num_test_repeats--)
    // {

    // Create config system, loading 'defaultConfig'.
    startTimedTest();
    // ConfigManager config("Testing_MMConfig");
    config = new ConfigManager("Testing_MMConfig");
    endTimedTest();
    std::cout << "Load configuration." << std::endl;
    std::cout << std::endl;

    // Declare test variables:
    bool result;
    double                                          store_d    , read_d    ;
    long                                            store_l    , read_l    ;
    std::vector<double>                             store_1dv_d, read_1dv_d;
    std::vector<std::vector<double> >               store_2dv_d, read_2dv_d;
    std::vector<std::vector<std::vector<double> > > store_3dv_d, read_3dv_d;
    std::vector<long>                               store_1dv_l, read_1dv_l;
    std::vector<std::vector<long> >                 store_2dv_l, read_2dv_l;
    std::vector<std::vector<std::vector<long> > >   store_3dv_l, read_3dv_l;
    ConfigRange<double>                             store_r_d  , read_r_d  ;
    ConfigRange<long>                               store_r_l  , read_r_l  ;

    // Tests:
    // MODULE:
    std::vector<Configurable*> cfObjs;

    // Create config objects
    Module m1;
    Module m2;

    // Set Config Objects
    cfObjs.push_back(&m1);
    cfObjs.push_back(&m2);
    config->setConfigObjects(cfObjs);

    
    m1.doubleParam1 = -1;
    m2.doubleParam1 = -2;
    m1.setConfigBasePath("Testing.MM");
    m2.setConfigBasePath("dfhfgh");
    
    std::cout << "Module.doubleParam1 = " << m1.doubleParam1 << std::endl; 
    std::cout << "Module.doubleParam2 = " << m2.doubleParam1 << std::endl; 
    store_d = 5;
    std::cout << "Module.doubleParam1 = " << m1.doubleParam1 << std::endl; 
    std::cout << "Module.doubleParam2 = " << m2.doubleParam1 << std::endl; 
    std::cout << "config->storeDoubleValue(...)" << std::endl; 
    result = config->storeValue<double>("Testing.MM", "param_double", store_d);
    std::cout << "Module.doubleParam1 = " << m1.doubleParam1 << std::endl; 
    std::cout << "Module.doubleParam2 = " << m2.doubleParam1 << std::endl; 
    std::cout << "config->updateConfiguration(...)" << std::endl; 
    config->updateConfiguration();
    std::cout << "Module.doubleParam1 = " << m1.doubleParam1 << std::endl; 
    std::cout << "Module.doubleParam2 = " << m2.doubleParam1 << std::endl; 


    // DOUBLE:

    // Store a double:
    // pass: If stored successfully.
    // FAIL: If not stored.
    store_d = randReal();
    startTimedTest();
    result = config->storeDoubleValue("Testing.MM", "param_double", store_d);
    endTimedTest();

    // Read a double:
    // pass: If read successfully.
    // FAIL: If not read.
    startTimedTest();
    result &= config->readDoubleValue("Testing.MM", "param_double", read_d);
    endTimedTest();

    result &= (((float)store_d) == ((float)read_d)); // ignore rounding errors by comparig as floats
    printTestResult("storeDouble", result);
    printTestResult("readDouble" , result);
    std::cout << "    (stored '" << store_d 
              << "', read '"     << read_d 
              << "')" << std::endl;
    std::cout << std::endl;
    
    // Attempt to store a double as a long:
    // pass: If not stored (returning an error).
    // FAIL: If no error occurs (i.e. if returns true).
    startTimedTest();
    result = config->storeLongValue("Testing.MM", "param_double", store_l);
    endTimedTest();
    printTestResult("storeDoubleAsLong", !result);

    // Attempt to read a double as a long:
    // pass: If not read (returning an error).
    // FAIL: If no error occurs (i.e. if returns true).
    startTimedTest();
    result = config->readLongValue("Testing.MM", "param_double", read_l);
    endTimedTest();
    printTestResult("readDoubleAslong", !result);
    std::cout << std::endl;


    // DOUBLE RANGE:

    // Store a range<double>:
    // pass: If stored successfully.
    // FAIL: If not stored.

    // NOTE: Should generate a random range.
    store_r_d = ConfigRange<double>(5, 10, false, true, bt_closed, bt_closed);
    startTimedTest();
    result = config->storeDoubleRange("Testing.MM", "param_double", store_r_d);
    endTimedTest();

    // Read a range<double>:
    // pass: If read successfully.
    // FAIL: If not read.
    startTimedTest();
    result &= config->readDoubleRange("Testing.MM", "param_double", read_r_d);
    endTimedTest();

    result &= (store_r_d.getMin()            == read_r_d.getMin() &&
               store_r_d.getMax()            == read_r_d.getMax() &&
               store_r_d.getUpperBoundType() == read_r_d.getUpperBoundType() &&
               store_r_d.getLowerBoundType() == read_r_d.getLowerBoundType() &&
               store_r_d.getAutoClip()       == read_r_d.getAutoClip() &&
               store_r_d.getOutside()        == read_r_d.getOutside()
               );
    printTestResult("storeDoubleRange", result);
    printTestResult("readDoubleRange" , result);
    std::cout << std::endl;
    
    
    // Attempt to store a range<double> as a range<long>:
    // pass: If not stored (returning an error).
    // FAIL: If no error occurs (i.e. if returns true).
    startTimedTest();
    result = config->storeLongRange("Testing.MM", "param_double", store_r_l);
    endTimedTest();
    printTestResult("storeDoubleRangeAsLongRange", !result);
    
    // Attempt to read a range<double> as a range<long>:
    // pass: If not read (returning an error).
    // FAIL: If no error occurs (i.e. if returns true).
    startTimedTest();
    result = config->readLongRange("Testing.MM", "param_double", read_r_l);
    endTimedTest();
    printTestResult("readDoubleRangeAsLongRange", !result);
    std::cout << std::endl;



    // Tests for range violations
    #warning Must add range tests



    // Store a vector<double>:
    // pass: If stored successfully.
    // FAIL: If not stored.
    int  vecS = 5 + (randInt() % 5);
    store_1dv_d = std::vector<double>();
    for(int i = 0; i < vecS; i++)
    {
        store_1dv_d.push_back(randReal());
    }
    startTimedTest();
    result = config->storeDoubleVectorValue1D("Testing.MM", "param_vector1d_double", store_1dv_d);
    endTimedTest();

    // Read a vector<double>:
    // pass: If read successfully.
    // FAIL: If not read.
    startTimedTest();
    result &= config->readDoubleVectorValue1D("Testing.MM", "param_vector1d_double", read_1dv_d);
    endTimedTest();

    // std::cout << "    (stored '" << store_d 
    //           << "', read '"     << read_d 
    //           << "')" << std::endl;
    std::cout << "    Sizes: " << store_1dv_d.size() << " , " << read_1dv_d.size() << std::endl;

    result &= true;
    for(int i = 0; i < store_1dv_d.size(); i++)
    {
        if(i < read_1dv_d.size())
        {
            bool cmp = (((float)store_1dv_d[i]) == ((float)read_1dv_d[i])); // ignore rounding errors by comparig as floats
            result &= cmp;
            std::cout << "    " << (float)store_1dv_d[i] 
                      << " | "  << (float)read_1dv_d [i]
                      << (!cmp? " *" : "" ) << std::endl;
        }
        else result = false;
    }
    printTestResult("storeDoubleVectorValue1D", result);
    printTestResult("readDoubleVectorValue1D" , result);
    std::cout << std::endl;



    // Store a vector<vector<double> >:
    // pass: If stored successfully.
    // FAIL: If not stored.
    std::cout << " Create test vectors" << std::endl;
    int  vec2d_s1 = 3 + (randInt() % 3);
    store_2dv_d = std::vector<std::vector<double> >();
    for(int i = 0; i < vec2d_s1; i++)
    {
        int  vec2d_s2 = 0 + (randInt() % 5);
        std::vector<double> v = std::vector<double>();
        for(int i = 0; i < vec2d_s2; i++)
        {
            v.push_back(randReal());
        }
        store_2dv_d.push_back(v);
    }
    startTimedTest();
    result = config->storeDoubleVectorValue2D("Testing.MM", "param_vector2d_double", store_2dv_d);
    std::cout << (result? "T":"F" ) << std::endl;
    endTimedTest();

    // Read a vector<vector<double> >:
    // pass: If read successfully.
    // FAIL: If not read.
    startTimedTest();
    result &= config->readDoubleVectorValue2D("Testing.MM", "param_vector2d_double", read_2dv_d);
    std::cout << (result? "T":"F" ) << std::endl;
    endTimedTest();

    std::cout << " Compare" << std::endl;
    
    std::cout << "  Sizes: " << store_2dv_d.size() 
              << " , "       << read_2dv_d .size() 
              << std::endl;

    result &= true;
    for(int i = 0; i < store_2dv_d.size(); i++)
    {
        if(!(i < read_2dv_d.size())) { result = false; break; }
        std::cout << "    Sizes: " << store_2dv_d[i].size() 
                  << " , "         << read_2dv_d [i].size()
                  << std::endl;

        for(int j = 0; j < store_2dv_d[i].size(); j++)
        {
            if(!(j < read_2dv_d[i].size())) { result = false; break; }

            bool cmp = (((float)store_2dv_d[i][j]) == ((float)read_2dv_d[i][j])); // ignore rounding errors by comparig as floats
            result &= cmp;
            std::cout << "      " << (float)store_2dv_d[i][j] 
                      << " | "    << (float) read_2dv_d[i][j]
                      << (!cmp? " *" : "" ) << std::endl;
        }
    }
    printTestResult("storeDoubleVectorValue2D", result);
    printTestResult("readDoubleVectorValue2D" , result);
    std::cout << std::endl;



    // Store a vector<vector<vector<double> > >:
    // pass: If stored successfully.
    // FAIL: If not stored.
    std::cout << " Create test vectors" << std::endl;
    int  vec3d_s1 = 3 + (randInt() % 2);
    store_3dv_d = std::vector<std::vector<std::vector<double> > >();
    for(int i = 0; i < vec3d_s1; i++)
    {
        int  vec3d_s2 = 0 + (randInt() % 5);
        std::vector<std::vector<double> > v2 = std::vector<std::vector<double> >();
        for(int i = 0; i < vec3d_s2; i++)
        {
            int  vec3d_s3 = 0 + (randInt() % 5);
            std::vector<double> v1 = std::vector<double>();
            for(int i = 0; i < vec3d_s3; i++)
            {
                v1.push_back(randReal());
            }
            v2.push_back(v1);
        }
        store_3dv_d.push_back(v2);
    }
    startTimedTest();
    result = config->storeDoubleVectorValue3D("Testing.MM", "param_vector3d_double", store_3dv_d);
    endTimedTest();
    std::cout << (result? "T":"F" ) << std::endl;


    // Read a vector<vector<vector<double> > >:
    // pass: If read successfully.
    // FAIL: If not read.
    startTimedTest();
    result &= config->readDoubleVectorValue3D("Testing.MM", "param_vector3d_double", read_3dv_d);
    endTimedTest();

    std::cout << "Compare" << std::endl;
    
    std::cout << "  Sizes: " << store_3dv_d.size() 
              << " , "         << read_3dv_d .size() 
              << std::endl;

    result &= true;
    for(int i = 0; i < store_3dv_d.size(); i++)
    {
        if(!(i < read_3dv_d.size())) { result = false; break; }
        std::cout << "    Sizes: "   << store_3dv_d[i].size() 
                  << " , "           << read_3dv_d [i].size()
                  << std::endl;

        for(int j = 0; j < store_3dv_d[i].size(); j++)
        {
            if(!(j < read_3dv_d[i].size())) { result = false; break; }
            std::cout << "      Sizes: "   << store_3dv_d[i][j].size() 
                      << " , "             << read_3dv_d [i][j].size()
                      << std::endl;

            for(int k = 0; k < store_3dv_d[i][j].size(); k++)
            {
                if(!(k < read_3dv_d[i][j].size())) { result = false; break; }

                bool cmp = (((float)store_3dv_d[i][j][k]) == ((float)read_3dv_d[i][j][k])); // ignore rounding errors by comparig as floats
                result &= cmp;
                std::cout << "        " << (float)store_3dv_d[i][j][k] 
                          << " | "      << (float) read_3dv_d[i][j][k]
                          << (!cmp? " *" : "" ) << std::endl;
            }
        }
    }
    printTestResult("storeDoubleVectorValue3D", result);
    printTestResult("readDoubleVectorValue3D" , result);
    std::cout << std::endl;


    // // LONG:

    // // Store a long:
    // // pass: If stored successfully.
    // // FAIL: If not stored.
    // store_l = randInt();
    // startTimedTest();
    // result = config->storeLongValue("Testing.MM", "param_long", store_l);
    // endTimedTest();

    // // Read a long:
    // // pass: If read successfully.
    // // FAIL: If not read.
    // startTimedTest();
    // result = config->readLongValue("Testing.MM", "param_long", read_l);
    // endTimedTest();

    // result = (store_l == read_l);
    // printTestResult("storeLong", result);
    // printTestResult("readLong", result);
    // std::cout << std::endl;





    // Save configuration as 'newConfig'.
    startTimedTest();
    config->saveConfiguration("newConfig");
    endTimedTest();
    std::cout << "Save configuration." << std::endl;
    std::cout << std::endl;
    
    // // A property tree that holds ConfigParameter objects
    // boost::property_tree::basic_ptree<std::string, ConfigParameter, std::less<std::string> >
    //  cpTree;
    // ConfigParameter newCP(vt_double);
    // cpTree.put("root", newCP);


    // } // repeat all tests...


    delete config; config = NULL;

    return 0;
}
