#ifndef HISTOGRAM1D_H
#define HISTOGRAM1D_H

#include <vector>

using std::vector;
using std::pair;

//! @todo Make this a templated class and keep a track of objects for later use

struct Bin
{
    double value;
    double start;
    double width;
};

class Histogram1D
{
public:
    Histogram1D(size_t num_bins, double bin_width);
    Histogram1D(vector<double> bin_widths);

    static Bin emptyBin(double start, double width);

    //accessors
    Bin getBin(double pos);
    size_t getBinIndex(double pos);
    vector<Bin> getLargestBins(size_t n);
    vector<Bin>::const_iterator begin() const {return m_bins.begin();}
    vector<Bin>::const_iterator end() const {return m_bins.end();}

    //mutators
    void addToBin(double pos, double val);
    void addToBins(double start, double end, double density);
    void mergeBins(Bin first, Bin second);
    void mergeAdjacentPeaks(double minimum_size);


private:
    vector<Bin> m_bins;
    double m_end;
};

#endif // HISTOGRAM1D_H
