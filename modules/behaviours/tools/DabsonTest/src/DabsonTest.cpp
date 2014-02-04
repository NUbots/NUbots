#include "DabsonTest.h"

#include "utility/math/kalman/UKF.h"
#include "utility/math/kalman/SinModel.h"

#include "utility/NUbugger/NUgraph.h"
using utility::NUbugger::graph;

unsigned int i = 0; //TESTING_HERE

namespace modules {
    namespace behaviours {
        namespace tools {

            DabsonTest::DabsonTest(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
                
                // Get the scripts to run from the command line
                on<Trigger<Every<10, std::chrono::milliseconds>>>([this](const time_t& time) {
                    
                    double freq = 0.25;

                    double t = time.time_since_epoch().count() / double(NUClear::clock::period::den);

                    float s = sin(2 * M_PI * freq * t);

                    s += (rand()/double(RAND_MAX) - 0.5) * 0.2;

                    // Do kalman filtery stuff in here

                    UKF<SinModel> k;

                    arma::mat something;

                    k.timeUpdate(100, something, something, something);
                    k.measurementUpdate(something, something, something);

                    float o = s; //o ~ the kalman filter output.

                    emit(graph("Filter", s, o)); //plot the sinewave & the kalmanfilter output
                });
            }
            
        }  // tools
    }  // behaviours
}  // modules


/*
#include <iostream> 
#include <stdlib.h> //opting for printf() in lieu of std::cout
#include <time.h> //for seeding random
#include <math.h> //
#include "Kalman.h"
#include "IMUModel.h"
#include <armadillo>

//using namespace std;
using std::cout;
using std::endl;

const int TESTFIDELITY = 500; //size of
const double ANGULAR_FREQ = 0.05;

int main() {      
    std::cout << "Kalman testing" << std::endl;
    //IMUModel m; //yes, pointers arnt supposed to be used in
    Kalman<IMUModel> k; //only use () when a constructor accepts a parameter!!!!

    //m.test();
    
      //testing the armadillo libraries are working
        arma::mat A(3,4); A = "1 2 3 4;5 6 7 8; 9 10 11 0;";
    arma::mat B(3,4); B = "1 2 3 4;5 6 7 8; 5 10 11 0;";
    //arma::mat C(3,4); C.zeros();
    //std::cout << A << "\n" << B << std::endl;
    std::cout << arma::vectorise(A==B) << std::endl;
    std::cout << "Are the matrices equal?      " << arma::all(arma::all(A==B)) << std::endl;
    std::cout << "Are the matricies different? " << arma::any(arma::vectorise(A!=B)) << std::endl;
    
    
  
    //---------------------------------------------------------------------------------------------------------------
    //return 0; //when your ready for the actual tests, uncomment this line
    srand(time(0));
    std::cout << "Testing the kalman filter by sending in a sine wave, adding noise, then filtering it." << std::endl;

    double sinewave[TESTFIDELITY];
    double noisysin[TESTFIDELITY];
    double filteredsin[TESTFIDELITY];
    
    double dela = 0.; //the average delta from the unfiltered result
    double delb = 0.; //the average detla from the filtered result
    double tmp  = 0.;
    
    for(int i=0;i<TESTFIDELITY;i++) {
        double trueX = ANGULAR_FREQ*i;
        
        tmp = (9000-(rand()%18000))/1000.; //the random noise
        tmp /= 10.; //instead of +-9.0 units of noise, lets just go with +-0.9 to start with
        sinewave[i]    = 90.*sin(trueX);
        noisysin[i]    = sinewave[i] + tmp; //90+-9.000 for noise
        
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ test!!!
        
        //@brief Measurement update function The measurement update function corrects the estimated state of the system using observed measurement/s. 
    //@param measurement The measured data to be used for the update. 
    //@param noise The noise associated with the measurement data provided. 
    //@param args Any additional arguments required for the measurement update. 
    //@param type The type of measurement. @return True if the update was performed sucessfully. False if the update was unable to be performed.
    //bool measurementUpdate(const arma::mat& measurement, const arma::mat& noise, const arma::mat& args, unsigned int type); 
        //kf::filter(mymodel, noisysin[i]);
        filteredsin[i] = noisysin[i]; //change this with a kalman filter ie. mymodel.x(0,0);
        
        arma::mat a(1,1); ////setting up the tests...................
        a << noisysin[i];
        
        //a ~ measurement
        //b ~ noise
        //c ~ args
        //d ~ type (type of measurement)
        
        arma::mat b(1, 1);
        b << 0.9;
        
        arma::mat c(1, 1);
        c << 0;
        int d=0;
        
        //const arma::mat& u = a; //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~+++
        //const arma::mat& v = b; //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~+++
        //const arma::mat& w = c; //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~+++
        
            //
           // the last argument (type) is one of 3 types (defined in IMUModel.h)
         //  bbbb build me
          //enum MeasurementType {
         //   kmeasurement_accelerometer,
          //  kmeasurement_kinematic,
         //   kmeasurement_total_types
          //};
        //     *
        
        arma::mat q(1,1);
        
        //const arma::mat& measurement, const arma::mat& noise, const arma::mat& args, unsigned int type
        //k.measurementUpdate(a,b,c,d); //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        
        std::cout << "measurmentUpdate? " << d << std::endl;
        
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        
        printf("%+07.3lf  %+07.3lf  %+07.3lf  %+07.3lf  %+07.3lf\n", trueX, sinewave[i], noisysin[i], filteredsin[i], std::abs(tmp));
        dela += std::abs(sinewave[i] - noisysin[i]); //be wary of loopround errors! :3
        delb += std::abs(sinewave[i] - filteredsin[i]);
    }
    
    dela /= TESTFIDELITY;
    delb /= TESTFIDELITY;
    
    cout << endl;
    cout << "Average noise:      " << dela << endl;
    cout << "Filtered deviation: " << delb << endl;

    //std::cout << "a" << k << "z" << std::endl;
    cout << endl << endl;
    return 0;
}


*/