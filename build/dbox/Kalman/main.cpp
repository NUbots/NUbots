#include <iostream> //I never really was on your side, c++
#include <stdlib.h> //opting for printf() in lieu of std::cout
#include <time.h> //for seeding random
#include <math.h> //
#include "Kalman.h"
#include "IMUModel.h"
#include <armadillo>

//using namespace std;
using std::cout;
using std::endl;

const unsigned int TESTFIDELITY = 500; //size of
const double ANGULAR_FREQ = 0.05;

int main() {  
    
    std::cout << "Kalman testing :)" << std::endl;
    IMUModel m; //yes, pointers arnt supposed to be used in
    Kalman<IMUModel> k; //only use () when a constructor accepts a parameter!!!!

    //m.test();
    
    /*  //testing the armadillo libraries are working
    	arma::mat A(3,4); A = "1 2 3 4;5 6 7 8; 9 10 11 0;";
	arma::mat B(3,4); B = "1 2 3 4;5 6 7 8; 5 10 11 0;";
	//arma::mat C(3,4); C.zeros();
	//std::cout << A << "\n" << B << std::endl;
	std::cout << arma::vectorise(A==B) << std::endl;
	std::cout << "Are the matrices equal?      " << arma::all(arma::all(A==B)) << std::endl;
	std::cout << "Are the matricies different? " << arma::any(arma::vectorise(A!=B)) << std::endl;
    */
    
  
    //---------------------------------------------------------------------------------------------------------------
    return 0; //when your ready for the actual tests, uncomment this line
    srand(time(0));
    std::cout << "Testing the kalman filter by sending in a sine wave, adding noise, then filtering it." << std::endl;

    double sinewave[TESTFIDELITY];
    double noisysin[TESTFIDELITY];
    double filteredsin[TESTFIDELITY];
    
    double avg  = 0.;
    double dela = 0.; //the average delta from the unfiltered result
    double delb = 0.; //the average detla from the filtered result
    double tmp  = 0.;
    
    for(int i=0;i<TESTFIDELITY;i++) {
        double trueX = ANGULAR_FREQ*i;
        
        tmp = (9000-(rand()%18000))/1000.; //the random noise
        sinewave[i]    = 90.*sin(trueX);
        noisysin[i]    = sinewave[i] + tmp; //90+-9.000 for noise
        
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ test!!!
        //kf::filter(mymodel, noisysin[i]);
        filteredsin[i] = noisysin[i]; //change this with a kalman filter ie. mymodel.x(0,0);
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        
        printf("%+07.3lf  %+07.3lf  %+07.3lf  %+07.3lf  %+07.3lf\n", trueX, sinewave[i], noisysin[i], filteredsin[i], std::abs(tmp));
        avg  += std::abs(tmp);
        dela += std::abs(sinewave[i] - noisysin[i]); //be wary of loopround errors! :3
        delb += std::abs(sinewave[i] - filteredsin[i]);
    }
    
    avg  /= TESTFIDELITY;
    dela /= TESTFIDELITY;
    delb /= TESTFIDELITY;
    
    cout << endl;
    cout << "avg                 " << avg  << endl; // sanity check - should be the same as dela
    cout << "Average noise:      " << dela << endl;
    cout << "Filtered deviation: " << delb << endl;

    //std::cout << "a" << k << "z" << std::endl;
    cout << endl << endl;
    return 0;
}

