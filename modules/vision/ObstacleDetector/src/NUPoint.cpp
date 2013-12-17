#include "NUPoint.h"

NUPoint::NUPoint() : screenCartesian(arma::zeros<vec>(2)), screenAngular(arma::zeros<vec>(2)), groundCartesian(arma::zeros<vec>(2)), neckRelativeRadial(arma::zeros<vec>(3)) {
	// Empty constructor.
}

NUPoint::NUPoint(const arma::vec2& screenCartesian, const arma::vec2& screenAngular, const arma::vec2& groundCartesian, const arma::vec3& neckRelativeRadial) {
	m_screenCartesian = screenCartesian;
	m_screenAngular = screenAngular;
	m_groundCartesian = groundCartesian;
	m_neckRelativeRadial = neckRelativeRadial;
}

arma::vec2& NUPoint::getScreenCartesian() const {
	return m_screenCartesian;
}

arma::vec2& NUPoint::getScreenAngular() const {
	return m_screenAngular;
}

arma::vec2& NUPoint::getGroundCartesian() const {
	return m_groundCartesian;
}

arma::vec3& NUPoint::getNeckRelativeRadial() const {
	return m_neckRelativeRadial;
}

std::ostream& operator<< (std::ostream& stream, const NUPoint& point) {
    stream << point.getScreenCartesian << "  " << point.getScreenAngular << "  " << point.getGroundCartesian << "  " << point.getNeckRelativeRadial;

    return stream;
}
