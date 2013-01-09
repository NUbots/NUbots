#include <queue>
#include <stdexcept>
#include <boost/foreach.hpp>
#include "histogram1D.h"
#include "debug.h"

class BinComparison
{
public:
  bool operator() (const Bin& l, const Bin&r) const
  {
    return (l.value >= r.value); //order by value not index
  }
};

Histogram1D::Histogram1D(int num_bins, int bin_width)
{
    m_bins.clear();
    for(int i=0; i<num_bins; i++) {
        m_bins.push_back(emptyBin(i*bin_width, bin_width));
    }
}


Histogram1D::Histogram1D(vector<int> bin_widths)
{
    int cur_pos = 0;
    m_bins.clear();
    for(int i=0; i<bin_widths.size(); i++) {
        m_bins.push_back(emptyBin(cur_pos, bin_widths.at(i)));
        cur_pos += bin_widths.at(i);
    }
}

Bin Histogram1D::emptyBin(int start, int width)
{
    Bin b;
    b.start = start;
    b.width = width;
    b.value = 0;
    return b;
}

Bin Histogram1D::getBin(int pos) {
    return m_bins.at(getBinIndex(pos));
}

unsigned int Histogram1D::getBinIndex(int pos)
{
    //adds the given element to the matching bin unless out of range, throws std::out_of_range
    if(pos < 0) {
        throw std::out_of_range("Histogram1D::addToBin called with negative position.");
    }

    for(unsigned int i=0; i<m_bins.size(); i++) {
        if(pos < m_bins.at(i).start + m_bins.at(i).width) {
            return i;
        }
    }

    throw std::out_of_range("Histogram1D::addToBin called with position too high.");
}

vector<Bin> Histogram1D::getLargestBins(unsigned int n)
{
    //O(nk)
    if(n < 1 || n > m_bins.size()) {
        throw std::out_of_range("Histogram1D::getLargestBins called for too many or too few bins.");
    }
    else {
        std::priority_queue<Bin, vector<Bin>, BinComparison> cur_largest;
        //push first n values
        for(unsigned int k=0; k<n; k++) {
            cur_largest.push(m_bins.at(k));
        }
        //go through rest of values
        for(unsigned int i=n; i<m_bins.size(); i++) {
            Bin smallest_held = cur_largest.top();
            Bin next = m_bins.at(i);
            if(next.value > smallest_held.value) {
                cur_largest.pop();
                cur_largest.push(next);
            }
        }

        //generate vector from priority queue
        vector<Bin> result;
        while(!cur_largest.empty()) {
            result.push_back(cur_largest.top());
            cur_largest.pop();
        }

        return result;
    }
}

void Histogram1D::mergeAdjacentPeaks(int minimum_size)
{
    vector<Bin>::iterator it = m_bins.begin();
    while(it + 1 != m_bins.end()) {
        if(it->value > minimum_size && (it+1)->value > minimum_size) {
            mergeBins(*it, *(it+1));
        }
        else {
            it++;
        }
    }
}

void Histogram1D::addToBin(int pos, int val)
{
    //adds the given element to the matching bin - if out_of_range will return false without modifying the histogram
    if(pos < 0) {
        throw std::out_of_range("Histogram1D::addToBin called with negative position.");
    }

    bool found = false;
    BOOST_FOREACH(Bin& bin, m_bins) {
        if(pos < bin.start + bin.width) {
            found = true;
            bin.value += val;
            return;
        }
    }

    throw std::out_of_range("Histogram1D::addToBin called with position too high.");
}

void Histogram1D::mergeBins(Bin first, Bin second)
{
    int first_index = getBinIndex(first.start),
        second_index = getBinIndex(second.start);

    if(first_index > second_index) {
        debug << "Histogram1D::mergeBins called with out of order positions, reordering and merging anyway." << std::endl;
        int temp = second_index;
        second_index = first_index;
        first_index = temp;
    }

    vector<Bin>::iterator it_first = m_bins.begin() + first_index;
    vector<Bin>::iterator it_next;

    for(it_next = it_first + 1; it_next != m_bins.begin() + second_index + 1; it_next++) {
        it_first->value += it_next->value;
        it_first->width += it_next->width;
    }

    m_bins.erase(it_first+1, it_next);
}

