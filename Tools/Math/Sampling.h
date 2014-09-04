/**
 * @file Math/Sampling.h
 * Sampling schemes for n-d spaces.
 */


#ifndef SAMPLING_H
#define SAMPLING_H

#include <cmath>
#include <vector>
#include <iostream>


namespace Sampling
{

float HaltonPoint(int index = 0, int prime = 2) {
    float result = 0;
    float inverse = 1./prime;
    for (int i = index; i > 0; i = i/prime) {
        result = result + inverse*(i%prime);
        inverse /= prime;
    }
    return result;
}

std::vector<float> HaltonPointND(int index = 0, int dimensions = 3) {
    std::vector<float> result;
    float primes[26] = {2,3,5,7,11,13,17,19,23,29,31,37,41,43,47,53,59,61,67,71,73,79,83,89,97,101};
    for (int i = 0; i < dimensions; i++) {
        result.push_back(HaltonPoint(index,primes[i]));
    }
    return result;
}

float Covariance1D(std::vector<float>& x, std::vector<float>& y) {
    float result = 0.;
    
    for (int i = 0; i < x.size(); i++) {
        result += x[i]*y[i];
        float r2 = 0.;
        for (int j = 0; j < x.size(); j++) {
            r2 += x[i]*y[j];
        }
        result -= r2/x.size();
    }
    return result;
}

float Variance1D(std::vector<float>& x) {
    float result = 0.;
    float r2 = 0.;
    for (int i = 0; i < x.size(); i++) {
        result += x[i]*x[i];
        r2 += x[i];
    }
    return result - r2*r2/x.size();
}

}

#endif
