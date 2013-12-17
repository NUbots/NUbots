#ifndef NUPOINT_H
#define NUPOINT_H

#include <nuclear>
#include <armadillo>
#include <ostream>

std::ostream& operator<< (std::ostream& stream, const NUPoint& point);

class NUPoint {
public:
    NUPoint();
    NUPoint(const arma::vec2& screenCartesian, const arma::vec2& screenAngular, const arma::vec2& groundCartesian, const arma::vec3& neckRelativeRadial);

    arma::vec2& getScreenCartesian() const;
    arma::vec2& getScreenAngular() const;
    arma::vec2& getGroundCartesian() const;
    arma::vec3& getNeckRelativeRadial() const;

private:
    arma::vec2 screenCartesian;
    arma::vec2 screenAngular;
    arma::vec2 groundCartesian;
    arma::vec3 neckRelativeRadial;
};

#endif // NUPOINT_H
