/*! 
    @file TestConfig.cpp
    @brief Tests for the Config System.
  
    Things to test: (for a parameter of type T, not of type U)
      - Storing a value of type T to a parameter of type T
      - Reading a value of type T from a parameter of type T
      - Attempting to store type T in a param of type U
      - Attempting to read a value of type T from a param of type U
      - Attempt to read from a path+name that doesn't exist
      - 


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


template <typename T>
void test1DVector(const std::string &paramName)
{
    timeval t;
    gettimeofday(&t,NULL);
    boost::mt19937 seed( (int)t.tv_sec );

    boost::uniform_real<> distR(0, 1);
    boost::variate_generator<boost::mt19937&, boost::uniform_real<> > randReal(seed,distR);
    
    boost::uniform_int<> distI(0, 255);
    boost::variate_generator<boost::mt19937&, boost::uniform_int<> > randInt(seed,distI);

    std::vector<T> store_1dv_t, read_1dv_t;

    // Store a vector<T>:
    // pass: If stored successfully.
    // FAIL: If not stored.
    int  vecS = 5 + (randInt() % 5);
    store_1dv_t = std::vector<T>();
    for(int i = 0; i < vecS; i++)
    {
        store_1dv_t.push_back((T)(randReal() * 100));
    }
    startTimedTest();
    bool res_s = config->storeValue("Testing.MM", paramName, store_1dv_t);
    endTimedTest();

    // Read a vector<T>:
    // pass: If read successfully.
    // FAIL: If not read.
    startTimedTest();
    bool res_r = config->readValue("Testing.MM", paramName, read_1dv_t);
    endTimedTest();

    // std::cout << "    (stored '" << store_d 
    //           << "', read '"     << read_d 
    //           << "')" << std::endl;
    std::cout << "    Sizes: " << store_1dv_t.size() << " , " << read_1dv_t.size() << std::endl;

    bool res_c = true;
    for(int i = 0; i < store_1dv_t.size(); i++)
    {
        if(i < read_1dv_t.size())
        {
            bool cmp = (((float)store_1dv_t[i]) == ((float)read_1dv_t[i])); // ignore rounding errors by comparig as floats
            res_c &= cmp;
            std::cout << "    " << (float)store_1dv_t[i] 
                      << " | "  << (float)read_1dv_t [i]
                      << (!cmp? " *" : "" ) << std::endl;
        }
        else res_c = false;
    }

    bool result = res_s && res_r && res_c;
    printTestResult("storeVectorValue1D", result);
    printTestResult("readVectorValue1D" , result);
    std::cout << std::endl;
}


template <typename T>
void test2DVector(const std::string &paramName)
{
    timeval t;
    gettimeofday(&t,NULL);
    boost::mt19937 seed( (int)t.tv_sec );

    boost::uniform_real<> distR(0, 1);
    boost::variate_generator<boost::mt19937&, boost::uniform_real<> > randReal(seed,distR);
    
    boost::uniform_int<> distI(0, 255);
    boost::variate_generator<boost::mt19937&, boost::uniform_int<> > randInt(seed,distI);

    std::vector<std::vector<T> > read_2dv_t, store_2dv_t;

    // Store a vector<vector<T> >:
    // pass: If stored successfully.
    // FAIL: If not stored.
    std::cout << " Create test vectors" << std::endl;
    int  vec2d_s1 = 3 + (randInt() % 3);
    store_2dv_t = std::vector<std::vector<T> >();
    for(int i = 0; i < vec2d_s1; i++)
    {
        int  vec2d_s2 = 0 + (randInt() % 5);
        std::vector<T> v = std::vector<T>();
        for(int i = 0; i < vec2d_s2; i++)
        {
            v.push_back((T)(randReal() * 100));
        }
        store_2dv_t.push_back(v);
    }
    startTimedTest();
    bool res_s = config->storeValue("Testing.MM", paramName, store_2dv_t);
    std::cout << (res_s? "T":"F" ) << std::endl;
    endTimedTest();

    // Read a vector<vector<T> >:
    // pass: If read successfully.
    // FAIL: If not read.
    startTimedTest();
    bool res_r = config->readValue("Testing.MM", paramName, read_2dv_t);
    std::cout << (res_r? "T":"F" ) << std::endl;
    endTimedTest();

    std::cout << " Compare" << std::endl;
    
    std::cout << "  Sizes: " << store_2dv_t.size() 
              << " , "       << read_2dv_t .size() 
              << std::endl;

    bool res_c = true;
    for(int i = 0; i < store_2dv_t.size(); i++)
    {
        if(!(i < read_2dv_t.size())) { res_c = false; break; }
        std::cout << "    Sizes: " << store_2dv_t[i].size() 
                  << " , "         << read_2dv_t [i].size()
                  << std::endl;

        for(int j = 0; j < store_2dv_t[i].size(); j++)
        {
            if(!(j < read_2dv_t[i].size())) { res_c = false; break; }

            bool cmp = (((float)store_2dv_t[i][j]) == ((float)read_2dv_t[i][j])); // ignore rounding errors by comparig as floats
            res_c &= cmp;
            std::cout << "      " << (float)store_2dv_t[i][j] 
                      << " | "    << (float) read_2dv_t[i][j]
                      << (!cmp? " *" : "" ) << std::endl;
        }
    }

    bool result = res_s && res_r && res_c;
    printTestResult("storeVectorValue2D", result);
    printTestResult("readVectorValue2D" , result);
    std::cout << std::endl;
}


template <typename T>
void test3DVector(const std::string &paramName)
{
    timeval t;
    gettimeofday(&t,NULL);
    boost::mt19937 seed( (int)t.tv_sec );

    boost::uniform_real<> distR(0, 1);
    boost::variate_generator<boost::mt19937&, boost::uniform_real<> > randReal(seed,distR);
    
    boost::uniform_int<> distI(0, 255);
    boost::variate_generator<boost::mt19937&, boost::uniform_int<> > randInt(seed,distI);

    std::vector<std::vector<std::vector<T> > > read_3dv_t, store_3dv_t;

    // Store a vector<vector<vector<T> > >:
    // pass: If stored successfully.
    // FAIL: If not stored.
    std::cout << " Create test vectors" << std::endl;
    int  vec3d_s1 = 3 + (randInt() % 2);
    store_3dv_t = std::vector<std::vector<std::vector<T> > >();
    for(int i = 0; i < vec3d_s1; i++)
    {
        int  vec3d_s2 = 0 + (randInt() % 5);
        std::vector<std::vector<T> > v2 = std::vector<std::vector<T> >();
        for(int i = 0; i < vec3d_s2; i++)
        {
            int  vec3d_s3 = 0 + (randInt() % 5);
            std::vector<T> v1 = std::vector<T>();
            for(int i = 0; i < vec3d_s3; i++)
            {
                v1.push_back((T)(randReal()*100));
            }
            v2.push_back(v1);
        }
        store_3dv_t.push_back(v2);
    }
    startTimedTest();
    bool res_s = config->storeValue("Testing.MM", paramName, store_3dv_t);
    endTimedTest();
    std::cout << (res_s? "T":"F" ) << std::endl;


    // Read a vector<vector<vector<T> > >:
    // pass: If read successfully.
    // FAIL: If not read.
    startTimedTest();
    bool res_r = config->readValue("Testing.MM", paramName, read_3dv_t);
    endTimedTest();
    std::cout << (res_r? "T":"F" ) << std::endl;

    std::cout << "Compare" << std::endl;
    
    std::cout << "  Sizes: " << store_3dv_t.size() 
              << " , "       << read_3dv_t .size() 
              << std::endl;

    bool res_c = true;
    for(int i = 0; i < store_3dv_t.size(); i++)
    {
        if(!(i < read_3dv_t.size())) { res_c = false; break; }
        std::cout << "    Sizes: "   << store_3dv_t[i].size() 
                  << " , "           << read_3dv_t [i].size()
                  << std::endl;

        for(int j = 0; j < store_3dv_t[i].size(); j++)
        {
            if(!(j < read_3dv_t[i].size())) { res_c = false; break; }
            std::cout << "      Sizes: "   << store_3dv_t[i][j].size() 
                      << " , "             << read_3dv_t [i][j].size()
                      << std::endl;

            for(int k = 0; k < store_3dv_t[i][j].size(); k++)
            {
                if(!(k < read_3dv_t[i][j].size())) { res_c = false; break; }

                bool cmp = (((float)store_3dv_t[i][j][k]) == ((float)read_3dv_t[i][j][k])); // ignore rounding errors by comparig as floats
                res_c &= cmp;
                std::cout << "        " << (float)store_3dv_t[i][j][k] 
                          << " | "      << (float) read_3dv_t[i][j][k]
                          << (!cmp? " *" : "" ) << std::endl;
            }
        }
    }

    bool result = res_s && res_r && res_c;
    printTestResult("storeVectorValue3D", result);
    printTestResult("readeVectorValue3D" , result);
    std::cout << std::endl;
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
    // m1.doubleParam1 should not equal 5 here
    std::cout << "Module.doubleParam1 = " << m1.doubleParam1 << std::endl; 
    std::cout << "Module.doubleParam2 = " << m2.doubleParam1 << std::endl; 
    std::cout << "config->updateConfiguration(...)" << std::endl; 
    config->updateConfiguration();
    // m1.doubleParam1 should equal 5 here
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

    result &= (store_r_d.getMin()            == read_r_d.getMin()            &&
               store_r_d.getMax()            == read_r_d.getMax()            &&
               store_r_d.getUpperBoundType() == read_r_d.getUpperBoundType() &&
               store_r_d.getLowerBoundType() == read_r_d.getLowerBoundType() &&
               store_r_d.getAutoClip()       == read_r_d.getAutoClip()       &&
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
    #warning Must add range tests (already tested by Sophie?)


    // Easy templated std::vector<...> tests!
    //
    // Desc: Each test generates a random value, writes it, then reads it,
    //       Then compares what was written to what was read (at a lower 
    //       resolution, to avoid failure due to rounding errors).
    //
    // Pass: passes only if both the read and write return success and the 
    //       written and read values are identical.

    std::cout << std::endl << "TEST: std::vector<double>" << std::endl;
    test1DVector<double>("param_vector1d_double");
    std::cout << std::endl << "TEST: std::vector<long>" << std::endl;
    test1DVector<long>("param_vector1d_long");


    std::cout << std::endl << "TEST: std::vector<std::vector<double> >" << std::endl;
    test2DVector<double>("param_vector2d_double");
    std::cout << std::endl << "TEST: std::vector<std::vector<long> >" << std::endl;
    test2DVector<long>("param_vector2d_long");
    

    std::cout << std::endl << "TEST: std::vector<std::vector<std::vector<double> > >" << std::endl;
    test3DVector<double>("param_vector3d_double");
    std::cout << std::endl << "TEST: std::vector<std::vector<std::vector<long> > >" << std::endl;
    test3DVector<long>("param_vector3d_long");


    bool res_del = config->deleteParam("Testing.MM", "param_double");
    std::cout << std::endl 
              << "TEST: deleteParam(...): " 
              << (res_del? "T" : "F") 
              << std::endl;
    res_del = config->deleteParam("Testing.MM", "param_double");
    std::cout << std::endl 
              << "TEST: deleteParam(...): " 
              << (res_del? "T" : "F") 
              << std::endl;
    
    config->createParam<double>("Testing.MM", "param_double", 10);

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
