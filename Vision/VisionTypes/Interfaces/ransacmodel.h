#ifndef RANSACMODEL_H
#define RANSACMODEL_H

template<typename DataPoint>
class RANSACModel {
public:
    virtual ~RANSACModel();

    virtual bool regenerate(const vector<DataPoint>& pts) = 0;

    virtual unsigned int minPointsForFit() const = 0;

    virtual double calculateError(DataPoint p) const = 0;
};

#endif // RANSACMODEL_H
