#ifndef HISTOGRAM1D_H
#define HISTOGRAM1D_H

#include <vector>

using std::vector;
using std::pair;

struct Bin
{
    int value;
    int start;
    int width;
};

class Histogram1D
{
public:
    static Bin emptyBin(int start, int width);

    Histogram1D(int num_bins, int bin_width);
    Histogram1D(vector<int> bin_widths);

    //accessors
    Bin getBin(int pos);
    unsigned int getBinIndex(int pos);
    vector<Bin> getLargestBins(unsigned int n);

    //mutators
    void addToBin(int pos, int val);
    void mergeBins(Bin first, Bin second);
    void mergeAdjacentPeaks(int minimum_size);


private:
    vector<Bin> m_bins;
};

#endif // HISTOGRAM1D_H
