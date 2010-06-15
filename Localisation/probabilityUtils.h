
#ifndef PROBABILITYUTILS_H
#define PROBABILITYUTILS_H

/**
	@class ProbabilityUtils
	@author Shashank Bhatia
	A class to implement Standard Probability tools like Sample Extraction 
	from normal and uniform distributions.
	TODO Implementation of PCA, MLE, EM and MAP
*/
class ProbabilityUtils{

	
	public:
		/// Constructor
		ProbabilityUtils();

        float normalDistribution(float mean, float sigma);
		/// Function to Draw Random Sample from Gaussian Distribution
		double randomGaussian(double mean, double stdDev);
		
		/// Function to Draw Random Sample from Uniform Distribution
		double randomUniform(double minLimit, double maxLimit);
		
		/// Function to calculate Probability of Value in a zero centered Distribution with given variance
		double ProbabilityOfValInVariance(double Val,double range);
		
		/// Destructor
		~ProbabilityUtils();
		
		double normalRandom(double mean, double stdDev);

};

#endif
