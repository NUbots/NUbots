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
#include <sstream>
#include <vector>
#include <exception>
#include <boost/random/uniform_int.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>

#include <boost/format.hpp> 

#include <sys/time.h>

#include "ConfigManager.h"
#include "ConfigRange.h"
//#include "ConfigStorageManager.h"

#include "Module.h"

using namespace ConfigSystem;

ConfigManager* config;


boost::mt19937 seed(0);

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
void create1DTestVector(
    std::vector<T> &vec_out,
    int s_min = 0, 
    int s_max = 5,
    T v_min = 0,
    T v_max = 100
    )
{
    // timeval t;
    // gettimeofday(&t,NULL);
    // boost::mt19937 seed( (int)t.tv_sec );
    boost::uniform_real<> distR(v_min, v_max);
    boost::variate_generator<boost::mt19937&, boost::uniform_real<> > randReal(seed,distR);
    boost::uniform_int<> distI(s_min, s_max);
    boost::variate_generator<boost::mt19937&, boost::uniform_int<> > randInt(seed,distI);

    vec_out = std::vector<T>();
    
    int  vecS = randInt();
    for(int i = 0; i < vecS; i++)
    {
        T val = (T)randReal();
        vec_out.push_back(val);
    }
}
template <typename T>
void create2DTestVector(
    std::vector<std::vector<T> > &vec_out,
    int s_min = 0, 
    int s_max = 10,
    T v_min = 0,
    T v_max = 100
    )
{
    // timeval t;
    // gettimeofday(&t,NULL);
    // boost::mt19937 seed( (int)t.tv_sec );
    boost::uniform_real<> distR(v_min, v_max);
    boost::variate_generator<boost::mt19937&, boost::uniform_real<> > randReal(seed,distR);
    boost::uniform_int<> distI(s_min, s_max);
    boost::variate_generator<boost::mt19937&, boost::uniform_int<> > randInt(seed,distI);

    vec_out = std::vector<std::vector<T> >();
    
    int  vecS = randInt();
    for(int i = 0; i < vecS; i++)
    {
        std::vector<T> val;
        create1DTestVector(val, s_min, s_max, v_min, v_max);
        vec_out.push_back(std::vector<T>(val));
    }
}
template <typename T>
void create3DTestVector(
    std::vector<std::vector<std::vector<T> > > &vec_out,
    int s_min = 0, 
    int s_max = 5,
    T v_min = 0,
    T v_max = 100
    )
{
    // timeval t;
    // gettimeofday(&t,NULL);
    // boost::mt19937 seed( (int)t.tv_sec );
    boost::uniform_real<> distR(v_min, v_max);
    boost::variate_generator<boost::mt19937&, boost::uniform_real<> > randReal(seed,distR);
    boost::uniform_int<> distI(s_min, s_max);
    boost::variate_generator<boost::mt19937&, boost::uniform_int<> > randInt(seed,distI);

    vec_out = std::vector<std::vector<std::vector<T> > >();
    
    int  vecS = randInt();
    for(int i = 0; i < vecS; i++)
    {
        std::vector<std::vector<T> > val;
        create2DTestVector(val, s_min, s_max, v_min, v_max);
        vec_out.push_back(std::vector<std::vector<T> >(val));
    }
}

template <typename T>
bool compare1DVectors(std::vector<T> u, std::vector<T> v, int indent = 0)
{
    std::stringstream ss;
    for(int i = 0; i < indent; i ++)
    {
        ss.width(4);
        ss << "|";
    }
    std::string str_i = ss.str();

    bool res_c = true;

    int su = u.size();
    int sv = v.size();
    int s_max = (su > sv) ? su : sv;
    int s_min = (su < sv) ? su : sv;

    res_c &= (su == sv);

    std::cout << str_i << "(" << su << ", " << sv << ")" << std::endl;;
    for(int i = 0; i < s_max; i++)
    {
        std::cout << str_i;

        std::cout.width(8);
        if(i < su)  std::cout << u[i];
        else        std::cout << "_";
        
        std::cout << ", ";

        std::cout.width(8);
        if(i < sv)  std::cout << v[i];
        else        std::cout << "_";

        if(i < s_min)
        {
            // bool t_c = ((float)u[i] == (float)v[i]);
            bool t_c = ((long)(u[i] * 100000) == (long)(v[i] * 100000));
            res_c &= t_c;
            if(!t_c) std::cout << " *";
        }

        std::cout << std::endl;
    }

    return res_c;
}

template <typename T>
bool compare2DVectors(
    std::vector<std::vector<T> > u,
    std::vector<std::vector<T> > v,
    int indent = 0)
{
    std::stringstream ss;
    for(int i = 0; i < indent; i ++)
    {
        ss.width(4);
        ss << "|";
    }
    std::string str_i = ss.str();


    bool res_c = true;

    int su = u.size();
    int sv = v.size();
    int s_max = (su > sv) ? su : sv;
    int s_min = (su < sv) ? su : sv;

    res_c &= (su == sv);

    std::cout << str_i << "(" << su << ", " << sv << ")" << std::endl;
    for(int i = 0; i < s_max; i++)
    {
        std::vector<T> empty;
        std::cout << str_i << std::endl;
        if(i >=   su) compare1DVectors(empty, v[i], indent + 1);
        if(i >=   sv) compare1DVectors(u[i], empty, indent + 1);
        if(i < s_min) compare1DVectors(u[i], v[i], indent + 1);
    }

    return res_c;
}


template <typename T>
bool compare3DVectors(
    std::vector<std::vector<std::vector<T> > > u,
    std::vector<std::vector<std::vector<T> > > v,
    int indent = 0)
{
    std::stringstream ss;
    for(int i = 0; i < indent; i ++)
    {
        ss.width(4);
        ss << "|";
    }
    std::string str_i = ss.str();


    bool res_c = true;

    int su = u.size();
    int sv = v.size();
    int s_max = (su > sv) ? su : sv;
    int s_min = (su < sv) ? su : sv;

    res_c &= (su == sv);

    std::cout << str_i << "(" << su << ", " << sv << ")" << std::endl;;
    for(int i = 0; i < s_max; i++)
    {
        std::vector<std::vector<T> > empty;

        std::cout << str_i << std::endl;
        if(i >=   su) compare2DVectors(empty, v[i], indent + 1);
        if(i >=   sv) compare2DVectors(u[i], empty, indent + 1);
        if(i < s_min) compare2DVectors(u[i], v[i], indent + 1);
    }

    return res_c;
}


template <typename T>
bool testValue(const std::string &paramname, const std::string &wrongparamname)
{
    // timeval t;
    // gettimeofday(&t,NULL);
    // boost::mt19937 seed( (int)t.tv_sec );
    boost::uniform_real<> distR(0, 1);
    boost::variate_generator<boost::mt19937&, boost::uniform_real<> > randReal(seed,distR);
    boost::uniform_int<> distI(0, 255);
    boost::variate_generator<boost::mt19937&, boost::uniform_int<> > randInt(seed,distI);
    // std::vector<T> store_1dv_t, read_1dv_t;

    T store_t, read_t;
    // U store_u, read_u;

    
    // Store a double:
    // pass: If stored successfully.
    // FAIL: If not stored.
    store_t = (T)(randReal() * 100);
    startTimedTest();
    bool res_s = config->storeValue("Testing.MM", paramname, store_t);
    endTimedTest();
    
    // Read a double:
    // pass: If read successfully.
    // FAIL: If not read.
    startTimedTest();
    bool res_r = config->readValue("Testing.MM", paramname, read_t);
    endTimedTest();

    bool res_c = (((float)store_t) == ((float)read_t)); // ignore rounding errors by comparig as floats

    bool result = res_s && res_r && res_c;
    printTestResult("store", res_s);
    printTestResult("read" , res_r);
    printTestResult("compare" , res_c);
    std::cout << "    (stored '" << store_t 
              << "', read '"     << read_t 
              << "')" << std::endl;
    
    // Attempt to store a double as a long:
    // pass: If not stored (returning an error).
    // FAIL: If no error occurs (i.e. if returns true).
    startTimedTest();
    res_s = config->storeValue("Testing.MM", wrongparamname, store_t);
    endTimedTest();
    printTestResult("storeAsIncorrectType", !res_s);

    // Attempt to read a long as a double:
    // pass: If not read (returning an error).
    // FAIL: If no error occurs (i.e. if returns true).
    startTimedTest();
    res_r = config->readValue("Testing.MM", wrongparamname, read_t);
    endTimedTest();
    printTestResult("storeAsIncorrectType", !res_r);
    std::cout << std::endl;

    return result;
}


template <typename T>
bool test1DVector(const std::string &paramName)
{
    std::vector<T> store_1dv_t, read_1dv_t;


    create1DTestVector(store_1dv_t);

    startTimedTest();
    bool res_s = config->storeValue("Testing.MM", paramName, store_1dv_t);
    endTimedTest();

    // Read a vector<T>:
    // pass: If read successfully.
    // FAIL: If not read.
    startTimedTest();
    bool res_r = config->readValue("Testing.MM", paramName, read_1dv_t);
    endTimedTest();

    bool res_c = compare1DVectors(store_1dv_t, read_1dv_t);

    bool result = res_s && res_r && res_c;
    printTestResult("storeVectorValue1D", result);
    printTestResult("readVectorValue1D" , result);
    std::cout << std::endl;

    return result;
}


template <typename T>
bool test2DVector(const std::string &paramName)
{

    std::vector<std::vector<T> > read_2dv_t, store_2dv_t;

    // Store a vector<vector<T> >:
    // pass: If stored successfully.
    // FAIL: If not stored.
    create2DTestVector(store_2dv_t);

    startTimedTest();
    bool res_s = config->storeValue("Testing.MM", paramName, store_2dv_t);
    endTimedTest();
    std::cout << (res_s? "T":"F" ) << std::endl;

    // Read a vector<vector<T> >:
    // pass: If read successfully.
    // FAIL: If not read.
    startTimedTest();
    bool res_r = config->readValue("Testing.MM", paramName, read_2dv_t);
    endTimedTest();
    std::cout << (res_r? "T":"F" ) << std::endl;

    bool res_c = compare2DVectors(store_2dv_t, read_2dv_t);

    bool result = res_s && res_r && res_c;
    printTestResult("storeVectorValue2D", result);
    printTestResult("readVectorValue2D" , result);
    std::cout << std::endl;

    return result;
}


template <typename T>
bool test3DVector(const std::string &paramName)
{
    std::vector<std::vector<std::vector<T> > > read_3dv_t, store_3dv_t;

    // Store a vector<vector<vector<T> > >:
    // pass: If stored successfully.
    // FAIL: If not stored.
    create3DTestVector(store_3dv_t);

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

    bool res_c = compare3DVectors(store_3dv_t, read_3dv_t);

    bool result = res_s && res_r && res_c;
    printTestResult("storeVectorValue3D", result);
    printTestResult("readeVectorValue3D" , result);
    std::cout << std::endl;

    return result;
}



int main(void)
{
    timeval t;
    gettimeofday(&t,NULL);
    seed = boost::mt19937( (int)t.tv_sec );

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



    // Begin decent tests:
    bool res_all = true;

    // repeat tests
    for(int i = 0; i < 500 && res_all; i++)
    {
        // Easy templated value tests
        std::cout << std::endl << "TEST: double" << std::endl;
        res_all &= testValue<double>("param_double", "param_long");
        std::cout << std::endl << "TEST: long" << std::endl;
        res_all &= testValue<long>("param_long", "param_double");

        // Easy templated std::vector<...> tests!
        //
        // Desc: Each test generates a random value, writes it, then reads it,
        //       Then compares what was written to what was read (at a lower 
        //       resolution, to avoid failure due to rounding errors).
        //
        // Pass: passes only if both the read and write return success and the 
        //       written and read values are identical.
        std::cout << std::endl << "TEST: std::vector<double>" << std::endl;
        res_all &= test1DVector<double>("param_vector1d_double");
        std::cout << std::endl << "TEST: std::vector<long>" << std::endl;
        res_all &= test1DVector<long>("param_vector1d_long");


        std::cout << std::endl << "TEST: std::vector<std::vector<double> >" << std::endl;
        res_all &= test2DVector<double>("param_vector2d_double");
        std::cout << std::endl << "TEST: std::vector<std::vector<long> >" << std::endl;
        res_all &= test2DVector<long>("param_vector2d_long");
        

        std::cout << std::endl << "TEST: std::vector<std::vector<std::vector<double> > >" << std::endl;
        res_all &= test3DVector<double>("param_vector3d_double");
        std::cout << std::endl << "TEST: std::vector<std::vector<std::vector<long> > >" << std::endl;
        res_all &= test3DVector<long>("param_vector3d_long");
    }

    if(res_all) std::cout << std::endl << "Testing complete: All tests passed." << std::endl;
    else        std::cout << std::endl << "Testing complete: Some tests failed." << std::endl;
    std::cout << std::endl << std::endl;




    // Declare test variables:
    bool result;
    double                                          store_d    , read_d    ;
    long                                            store_l    , read_l    ;
    // std::vector<double>                             store_1dv_d, read_1dv_d;
    // std::vector<std::vector<double> >               store_2dv_d, read_2dv_d;
    // std::vector<std::vector<std::vector<double> > > store_3dv_d, read_3dv_d;
    // std::vector<long>                               store_1dv_l, read_1dv_l;
    // std::vector<std::vector<long> >                 store_2dv_l, read_2dv_l;
    // std::vector<std::vector<std::vector<long> > >   store_3dv_l, read_3dv_l;
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


    bool res_del = config->deleteParam("Testing", "MM");
    std::cout << std::endl 
              << "TEST: deleteParam(...) 1: " 
              << (res_del? "T" : "F") 
              << std::endl;
    res_del = config->deleteParam("Testing.MM", "param_double");
    std::cout << std::endl 
              << "TEST: deleteParam(...) 2: " 
              << (res_del? "T" : "F") 
              << std::endl;
    
    config->createParam<double>("Testing.MM", "param_double", 10);

    res_del = config->deleteParam("Testing.MM", "param_double");
    std::cout << std::endl 
              << "TEST: deleteParam(...) 3: " 
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
