/*
 * This file is part of FeatureDetector.
 *
 * FeatureDetector is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * FeatureDetector is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with FeatureDetector.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "CentreCircle.h"

namespace modules {
    namespace vision {

        CentreCircle::CentreCircle() {
            m_sizeOnScreen = arma::vec2(arma::zeros<arma::vec>(2));
            m_groundRadius = 0;
            valid = false;

            // Need more here.
        }

        CentreCircle::CentreCircle(const NUPoint& centre, double groundRadius, const arma::vec2& screenSize) {
            m_location = centre;
            m_sizeOnScreen = screenSize,
            m_groundRadius = groundRadius;
            valid = (m_location.neckRelativeRadial[0] > 0);
        }

        CentreCircle::~CentreCircle() {
            // Empty destructor.
        }

        bool CentreCircle::addToExternalFieldObjects(std::unique_ptr<messages::vision::CentreCircle> centreCircle) const {
			std::unique_ptr<messages::vision::CentreCircle> temp = std::unique_ptr<messages::vision::CentreCircle>(new messages::vision::CentreCircle());

            if (valid) {
                // Add centre circle to stationary field objects.
                temp->sphericalFromNeck = m_location.neckRelativeRadial;
                temp->sphericalError = m_sphericalError;
                temp->screenAngular = m_location.screenAngular;
                temp->screenCartesian = m_location.screenCartesian;
                temp->sizeOnScreen = m_sizeOnScreen;
                temp->timestamp = NUClear::clock::now();

                centreCircle = std::move(temp);
            }

            return valid;
        }

        //! @brief Calculation of error for optimisation
        double CentreCircle::findScreenError(VisionFieldObject* other) const {
            CentreCircle* c = dynamic_cast<CentreCircle*>(other);

            return (arma::norm(m_location.screenCartesian - c->m_location.screenCartesian, 2) + arma::norm(m_sizeOnScreen - c->m_sizeOnScreen, 2));
        }

        double CentreCircle::findGroundError(VisionFieldObject *other) const {
            CentreCircle* c = dynamic_cast<CentreCircle*>(other);

            return (arma::norm(m_location.groundCartesian - c->m_location.groundCartesian, 2) + abs(m_groundRadius - c->m_groundRadius));
        }

        double CentreCircle::getGroundRadius() const {
            return m_groundRadius;
        }

        std::ostream& operator<< (std::ostream& output, const CentreCircle& c) {
            output << "CentreCircle - " << std::endl;
            output << "\tpixelloc: " << c.m_location.screenCartesian << std::endl;
            output << "\tangularloc: " << c.m_location.screenAngular << std::endl;
            output << "\trelative field coords: " << c.m_location.neckRelativeRadial << std::endl;
            output << "\tspherical error: [" << c.m_sphericalError << "]" << std::endl;
            output << "\tsize on screen: [" << c.m_sizeOnScreen << "]";

            return output;
        }

        std::ostream& operator<< (std::ostream& output, const std::vector<CentreCircle>& circles) {
            for (const auto& circle : circles) {
                output << circle << std::endl;        
            }

            return output;
        }
            
    }
}
