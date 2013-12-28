#include "Obstacle.h"

namespace modules {
    namespace vision {

        using messages::vision::COLOUR_CLASS;

        Obstacle::Obstacle(arma::vec2 position, double width, double height, COLOUR_CLASS colour, const VisionKinematics& transformer) {
            m_id = OBSTACLE;
            m_location.screenCartesian = position;
            m_sizeOnScreen = arma::vec2({width, height});
            m_colour = colour;
            m_transformer = transformer;

            // CALCULATE DISTANCE AND BEARING VALS
            valid = (calculatePositions() && check());
        }

        bool Obstacle::addToExternalFieldObjects(std::unique_ptr<messages::vision::Obstacle> obstacle) const {
            std::unique_ptr<messages::vision::Obstacle> temp = std::unique_ptr<messages::vision::Obstacle>(new messages::vision::Obstacle());

            if (valid) {
                temp->sphericalFromNeck = m_location.neckRelativeRadial;
                temp->sphericalError = m_sphericalError;
                temp->screenAngular = m_location.screenAngular;
                temp->screenCartesian = m_location.screenCartesian;
                temp->sizeOnScreen = m_sizeOnScreen;
                temp->timestamp = NUClear::clock::now();

                obstacle = std::move(temp);

                return true;
            }

            else {
                return false;
            }
        }

        bool Obstacle::check() const {
            //! TODO: Do a check based on width and d2p consistency
        //    if (!distance_valid) {
        //        return false;
        //    }

            // All checks passed.
            return true;
        }

        double Obstacle::findScreenError(VisionFieldObject* other) const {
            Obstacle* obstacle = dynamic_cast<Obstacle*>(other);

            return (arma::norm(m_location.screenCartesian - obstacle->m_location.screenCartesian, 2) + 
                    arma::norm(m_sizeOnScreen - obstacle->m_sizeOnScreen, 2));
        }

        double Obstacle::findGroundError(VisionFieldObject* other) const {
            Obstacle* obstacle = dynamic_cast<Obstacle*>(other);
            double w = 2 * m_location.neckRelativeRadial[0] * tan(m_arcWidth * 0.5);              // w/2 = d * tan(theta/2)
            double w_o = 2 * obstacle->m_location.neckRelativeRadial[0] * tan(obstacle->m_arcWidth * 0.5);

            return (arma::norm(m_location.groundCartesian - obstacle->m_location.groundCartesian, 2)) + abs(w - w_obstacle);
        }

        bool Obstacle::calculatePositions() {
            // To the bottom of the Goal Post..
            m_transformer.calculateRepresentationsFromPixelLocation(m_location);

            // Find arc width.
            NUPoint gp1, gp2;
            arma::vec2 screenSize;
            screenSize << m_sizeOnScreen[0] << 0;

            gp1.screenCartesian = m_location.screenCartesian - screenSize;
            gp2.screenCartesian = m_location.screenCartesian + screenSize;

            m_transformer.calculateRepresentationsFromPixelLocation(gp1);
            m_transformer.calculateRepresentationsFromPixelLocation(gp2);

            m_arcWidth = std::abs(gp1.screenAngular[0] - gp2.screenAngular[0]);

            return (m_location.neckRelativeRadial[0] > 0);
        }

        //void Obstacle::render(cv::Mat &mat) const {
        //    arma::vec2 half = m_sizeOnScreen * 0.5;
        //    cv::line(mat, cv::Point2i(m_location_pixels.x-half.x, m_location_pixels.y-m_size_on_screen.y), cv::Point2i(m_location_pixels.x-half.x, m_location_pixels.y), cv::Scalar(255, 255, 0));
        //    cv::line(mat, cv::Point2i(m_location_pixels.x-half.x, m_location_pixels.y), cv::Point2i(m_location_pixels.x+half.x, m_location_pixels.y), cv::Scalar(255, 255, 0));
        //    cv::line(mat, cv::Point2i(m_location_pixels.x+half.x, m_location_pixels.y), cv::Point2i(m_location_pixels.x+half.x, m_location_pixels.y-m_size_on_screen.y), cv::Scalar(255, 255, 0));
        //}

        /*! @brief Stream insertion operator for a single ColourSegment.
         *      The segment is terminated by a newline.
         */
        std::ostream& operator<< (std::ostream& output, const Obstacle& obstacle) {
            output << "Obstacle" << std::endl;
            output << "\tpixelloc: " << obstacle.m_location.screenCartesian << std::endl;
            output << "\tangularloc: " << obstacle.m_location.screenAngular << std::endl;
            output << "\trelative field coords: " << obstacle.m_location.neckRelativeRadial << std::endl;

            return output;
        }

        /*! @brief Stream insertion operator for a vector of ColourSegments.
         *      Each segment is terminated by a newline.
         *  @relates ColourSegment
         */
        std::ostream& operator<< (std::ostream& output, const std::vector<Obstacle>& obstacles) {
            for (const auto& obstacle : obstacles) {
                output << obstacle << std::endl;
            }

            return output;
        }

    }
}
