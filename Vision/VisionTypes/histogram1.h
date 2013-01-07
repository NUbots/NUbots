#ifndef HISTOGRAM1_H
#define HISTOGRAM1_H

template<typename T>

class Histogram1
{
public:
    Histogram1(int num_bins, int bin_width);
    Histogram1(vector<int> bin_widths);

    T getBinVal(int bin_no);

    bool addToBin(int pos, T val);

    vector<int> getLargestBins(int n);

private:
    vector< pair<T, int> > m_bins;
};

#endif // HISTOGRAM1_H
