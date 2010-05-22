/***************************************************************************
File Name:	probabilityutils.cpp
PURPOSE:	Compilation of Probabilistic Tools to be made available as a
		standard for any Project

References:	Numerical Recipies in C ( MIT )
 ***************************************************************************/


#include "probabilityUtils.h"

#include <iostream>
#include <cstdlib>
#include <ctime>
#include <math.h>


ProbabilityUtils::ProbabilityUtils()
{
        srand((int)((time(NULL) +rand())));
}




/**
*  
* Function Name:	ProbabilityUtils
* Author:		Shashank Bhatia
* Description: 		Destructor
* 			
* Date: 		03-11-2008
* input: 		NA			
* Output:		NA
* 
*/
ProbabilityUtils::~ProbabilityUtils()
{
}




/**
*  
* Function Name:	randomGaussian
* Author:		Shashank Bhatia
* Description: 		random number generator 			
* input: 		mean and standard Deviation			
* Output:		random number from gaussian Distribution with
*			given mean and standard deviation
* @param 		mean 
* @param 		stdDev 
* @return 		random Sample
*/
double ProbabilityUtils::randomGaussian(double mean, double stdDev)
{
	

	const double norm = 1.0 / (RAND_MAX + 1.0);
	double u = 1.0 - rand() * norm;                  /* can't let u == 0 */
	double v = rand() * norm;
	double z = sqrt(-2.0 * log(u)) * cos(2.0 * M_PI * v);
	return (mean + stdDev * z);
	
	
}

double ProbabilityUtils::normalRandom(double mean, double stdDev)
{
	
	double res = 1 / sqrt(  2*M_PI* stdDev*stdDev );
	res = res * exp( (-1*mean*mean)/(2*stdDev*stdDev));
	
	return res;
/*
	const double norm = 1.0 / (RAND_MAX + 1.0);
	double u = 1.0 - rand() * norm;              
 	double v = rand() * norm;
 	double z = sqrt(-2.0 * log(u)) * cos(2.0 * M_PI * v);
 	return (mean + stdDev * z);*/
	
	
}



/**
*  
* Function Name:	randomUniform
* Author:		Shashank Bhatia
* Description: 		random number generator 			
* input: 		minimum and maximum limits
* Output:		random number from uniform Distribution with
*			given minimum and maximum limits
* @param 	minLimit
* @param 	maxLimit 
* @return 	random Sample
*/
double ProbabilityUtils::randomUniform(double minLimit, double maxLimit)
{
	return (minLimit + (maxLimit-minLimit)*(double(rand()) / double(RAND_MAX)));
}



/**
*  
* Function Name:	randomUniform
* Author:		Shashank Bhatia
* Description: 		probability calculator
* input: 		value and variance
* Output:		Probability of given value in a zero centered distribution with given variance
* @param 		minLimit
* @param 		maxLimit 
* @return 		probability of sample value
*/
double ProbabilityUtils::ProbabilityOfValInVariance(double val,double variance)
{
	


	
	double prob = (double)(exp   ( (-1)*(0.5)*val*val / fabs(variance))    ) *  (1/sqrt(2*M_PI*fabs(variance)));

	

	return prob;
	
}

