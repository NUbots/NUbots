#include "FIRFilter.h"
#include <assert.h>

FIRFilter::FIRFilter()
{
	m_num_taps = 0;
}

FIRFilter::FIRFilter(const std::vector<float>& coefficients)
{
	Setup(coefficients);
}

FIRFilter::~FIRFilter()
{
	m_sample_buffer.clear();
	m_coefficients.clear();
}

bool FIRFilter::Setup(const std::vector<float>& coefficients)
{
	int new_size = coefficients.size();
	assert(new_size > 1);
	if(new_size > 1)
	{
		m_num_taps = new_size;
		m_sample_buffer.clear();
		m_sample_buffer.resize(new_size);
		m_coefficients = coefficients;
		return true;
	}
	return false;
}

float FIRFilter::Update(float new_sample)
{
	m_sample_buffer.push_front(new_sample);
	return CalculateCurrentValue();
}

float FIRFilter::CalculateCurrentValue()
{
	float total_sum = 0;
	for(int n = 0; n < m_num_taps; ++n)
	{
		total_sum += m_coefficients[n] * m_sample_buffer[n];
	}
	return total_sum;
}
