#ifndef NUPOINT_H
#define NUPOINT_H

#include <nuclear>
#include <armadillo>
#include <ostream>


typedef struct {
    arma::vec2 screenCartesian;
    arma::vec2 screenAngular;
    arma::vec2 groundCartesian;
    arma::vec3 neckRelativeRadial;
} NUPoint;

std::ostream& operator<< (std::ostream& stream, const NUPoint& point) {
    stream << point.screenCartesian << "  " << point.screenAngular << "  " << point.groundCartesian << "  " << point.neckRelativeRadial;

    return stream;
}

#endif // NUPOINT_H
