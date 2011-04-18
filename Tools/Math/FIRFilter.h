/**
 * @file Math/FIRFilter.h
 *
 */
#ifndef FIRFILTER_H
#define FIRFILTER_H
#include <boost/circular_buffer.hpp>
#include <vector>
class FIRFilter
{
public:
	FIRFilter();
	FIRFilter(const std::vector<float>& coefficients);
	~FIRFilter();
	bool Setup(const std::vector<float>& coefficients);
	float Update(float new_sample);
private:
	float CalculateCurrentValue();
	int m_num_taps;
	boost::circular_buffer<float> m_sample_buffer;
	std::vector<float> m_coefficients;
};

#endif //FIRFILTER_H
