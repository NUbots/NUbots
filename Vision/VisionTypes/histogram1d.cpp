#include <queue>
#include <stdexcept>
#include <boost/foreach.hpp>
#include "histogram1d.h"
#include "debug.h"

class BinComparison
{
public:
  bool operator() (const Bin& l, const Bin&r) const
  {
    return (l.value >= r.value); //order by value not index
  }
};

Histogram1D::Histogram1D(size_t num_bins, double bin_width)
{
    m_bins.clear();
    for(size_t i=0; i<num_bins; i++) {
        m_bins.push_back(emptyBin(i*bin_width, bin_width));
    }
    m_end = num_bins*bin_width;
}


Histogram1D::Histogram1D(std::vector<double> bin_widths)
{
    double cur_pos = 0;
    m_bins.clear();
    for(size_t i=0; i<bin_widths.size(); i++) {
        m_bins.push_back(emptyBin(cur_pos, bin_widths.at(i)));
        cur_pos += bin_widths.at(i);
    }
    m_end = cur_pos;
}

Bin Histogram1D::emptyBin(double start, double width)
{
    Bin b;
    b.start = start;
    b.width = width;
    b.value = 0;
    return b;
}

Bin Histogram1D::getBin(double pos) {
    return m_bins.at(getBinIndex(pos));
}

size_t Histogram1D::getBinIndex(double pos)
{
    //adds the given element to the matching bin unless out of range, throws std::out_of_range
    if(pos < 0) {
        throw std::out_of_range("Histogram1D::addToBin called with negative position.");
    }

    for(size_t i=0; i<m_bins.size(); i++) {
        if(pos < m_bins.at(i).start + m_bins.at(i).width) {
            return i;
        }
    }

    throw std::out_of_range("Histogram1D::addToBin called with position too high.");
}

std::vector<Bin> Histogram1D::getLargestBins(size_t n)
{
    //O(nk)
    if(n < 1 || n > m_bins.size()) {
        throw std::out_of_range("Histogram1D::getLargestBins called for too many or too few bins.");
    }
    else {
        std::priority_queue<Bin, std::vector<Bin>, BinComparison> cur_largest;
        //push first n values
        for(size_t k=0; k<n; k++) {
            cur_largest.push(m_bins.at(k));
        }
        //go through rest of values
        for(size_t i=n; i<m_bins.size(); i++) {
            Bin smallest_held = cur_largest.top();
            Bin next = m_bins.at(i);
            if(next.value > smallest_held.value) {
                cur_largest.pop();
                cur_largest.push(next);
            }
        }

        //generate vector from priority queue
        std::vector<Bin> result;
        while(!cur_largest.empty()) {
            result.push_back(cur_largest.top());
            cur_largest.pop();
        }

        return result;
    }
}

void Histogram1D::mergeAdjacentPeaks(double minimum_size)
{
    std::vector<Bin>::iterator it = m_bins.begin();
    while(it + 1 != m_bins.end()) {
        if(it->value > minimum_size && (it+1)->value > minimum_size) {
            mergeBins(*it, *(it+1));
        }
        else {
            it++;
        }
    }
}

void Histogram1D::addToBin(double pos, double val)
{
    //adds the given element to the matching bin - if out_of_range will return false without modifying the histogram
    if(pos < 0) {
        throw std::out_of_range("Histogram1D::addToBin called with negative position.");
    }

    BOOST_FOREACH(Bin& bin, m_bins) {
        if(pos < bin.start + bin.width) {
            bin.value += val;
            return; //found so return
        }
    }

    throw std::out_of_range("Histogram1D::addToBin called with position too large.");
}

/// adds "density" to every position in [start, end)
void Histogram1D::addToBins(double start, double end, double density)
{
    //adds the given element to the matching bin - if out_of_range will return false without modifying the histogram
    if(start < 0)
        throw std::out_of_range("Histogram1D::addToBin called with negative position.");
    if(end > m_end)
        throw std::out_of_range("Histogram1D::addToBin called with position too large.");
    if(start > end)
        throw std::out_of_range("Histogram1D::addToBin called with inverted range (start > end).");

    std::vector<Bin>::iterator bs = m_bins.begin(),
           be,
           bi;
    //find first bin
    while(bs->start + bs->width <= start)   // inclusive start
        bs++;

    //find last bin
    be = bs;
    while(be->start + be->width < end)  // non inclusive end
        be++;

    if(be == bs) {
        //one bin - spread density across covered portion
        bs->value += density * (end - start);
    }
    else {
        //multiple bins
        //spread density across covered portion of first bin
        bs->value += density * (bs->start + bs->width - start);
        //spread density across covered portion of last bin
        be->value += density * (end - be->start);
        //spread density across full width of intermediate bins
        bi = bs;
        bi++;
        while(bi < be) {
            bi->value += density * bi->width;
            bi++;
        }
    }
}

void Histogram1D::mergeBins(Bin first, Bin second)
{
    int first_index = getBinIndex(first.start),
        second_index = getBinIndex(second.start);

    if(first_index > second_index) {
        #if VISION_GOAL_VERBOSITY > 1
            debug << "Histogram1D::mergeBins called with out of order positions, reordering and merging anyway." << std::endl;
        #endif
        int temp = second_index;
        second_index = first_index;
        first_index = temp;
    }

    std::vector<Bin>::iterator it_first = m_bins.begin() + first_index;
    std::vector<Bin>::iterator it_next;

    for(it_next = it_first + 1; it_next != m_bins.begin() + second_index + 1; it_next++) {
        it_first->value += it_next->value;
        it_first->width += it_next->width;
    }

    m_bins.erase(it_first+1, it_next);
}

