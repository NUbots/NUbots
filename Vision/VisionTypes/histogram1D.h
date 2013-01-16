#ifndef HISTOGRAM1D_H
#define HISTOGRAM1D_H

#include <vector>

using std::vector;
using std::pair;

//! @todo Make this a templated class and keep a track of objects for later use

struct Bin
{
    float value;
    float start;
    float width;
};

class Histogram1D
{
public:
    Histogram1D(int num_bins, float bin_width);
    Histogram1D(vector<float> bin_widths);

    static Bin emptyBin(float start, float width);

    //accessors
    Bin getBin(float pos);
    unsigned int getBinIndex(float pos);
    vector<Bin> getLargestBins(unsigned int n);
    vector<Bin>::const_iterator begin() const {return m_bins.begin();}
    vector<Bin>::const_iterator end() const {return m_bins.end();}

    //mutators
    void addToBin(float pos, float val);
    void mergeBins(Bin first, Bin second);
    void mergeAdjacentPeaks(float minimum_size);


private:
    vector<Bin> m_bins;
};

#endif // HISTOGRAM1D_H
