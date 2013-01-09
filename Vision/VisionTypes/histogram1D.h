#ifndef HISTOGRAM1D_H
#define HISTOGRAM1D_H

#include <vector>

using std::vector;
using std::pair;

//! @todo Make this a templated class and keep a track of objects for later use

struct Bin
{
    int value;
    int start;
    int width;
};

class Histogram1D
{
public:
    Histogram1D(int num_bins, int bin_width);
    Histogram1D(vector<int> bin_widths);

    static Bin emptyBin(int start, int width);

    //accessors
    Bin getBin(int pos);
    unsigned int getBinIndex(int pos);
    vector<Bin> getLargestBins(unsigned int n);
    vector<Bin>::const_iterator begin() const {return m_bins.begin();}
    vector<Bin>::const_iterator end() const {return m_bins.end();}

    //mutators
    void addToBin(int pos, int val);
    void mergeBins(Bin first, Bin second);
    void mergeAdjacentPeaks(int minimum_size);


private:
    vector<Bin> m_bins;
};

#endif // HISTOGRAM1D_H
