#include "FieldLine.h"

namespace modules {
    namespace vision {

        FieldLine::FieldLine(const LSFittedLine& screenLine, const LSFittedLine& groundLine) {
            m_id = FIELDLINE;
            set(screenLine, groundLine);
        }

        FieldLine::FieldLine(const arma::vec2<NUPoint>& endPoints) {
            m_id = FIELDLINE;
            set(endPoints);
        }

        void FieldLine::set(const LSFittedLine& screenLine, const LSFittedLine& groundLine) {
            m_screenLine = screenLine;
            m_groundLine = groundLine;

            if(screenLine.valid) {
                screenLine.getEndPoints(m_endPoints[0].screenCartesian, m_endPoints[1].screenCartesian);
            }

            else {
                m_endPoints[0].screenCartesian << -1 << -1;
                m_endPoints[1].screenCartesian << -1 << -1;   //-1 is an invalid pixel location
            }

            if(groundLine.valid) {
                groundLine.getEndPoints(m_endPoints[0].groundCartesian, m_endPoints[1].groundCartesian);
            }

            else{
                m_endPoints[0].groundCartesian << -1 << -1;
                m_endPoints[1].groundCartesian << -1 << -1;   //-1 is an impossible ground location
            }
        }

        void FieldLine::set(const arma::vec2<NUPoint>& endPoints) {
            m_screenLine.setLineFromPoints(endPoints[0].screenCartesian, endPoints[1].screenCartesian);
            m_groundLine.setLineFromPoints(endPoints[0].groundCartesian, endPoints[1].groundCartesian);
            m_endPoints = endPoints;
        }

        Line FieldLine::getScreenLineEquation() const {
            return m_screenLine;
        }

        Line FieldLine::getGroundLineEquation() const {
            return m_groundLine;
        }

        arma::vec2<NUPoint> FieldLine::getEndPoints() const {
            return m_endPoints;
        }

        bool FieldLine::addToExternalFieldObjects(FieldObjects *fieldobjects, float timestamp) const {
            return (false && fieldobjects && (timestamp == 0);
        }
          
        double FieldLine::findScreenError(VisionFieldObject* other) const {
            FieldLine* l = dynamic_cast<FieldLine*>(other);

            // Distances vary depending on endpoint assignment.
            double d1 = (arma::norm(m_endPoints[0].screenCartesian - l->m_endPoints[0].screenCartesian, 2) + 
                            arma::norm(m_endPoints[1].screenCartesian - l->m_endPoints[1].screenCartesian, 2));
            double d2 = (arma::norm(m_endPoints[0].screenCartesian - l->m_endPoints[1].screenCartesian, 2) +
                             arma::norm(m_endPoints[1].screenCartesian - l->m_endPoints[0].screenCartesian, 2));

            return std::min(d1, d2);
        }

        double FieldLine::findGroundError(VisionFieldObject* other) const {
            FieldLine* l = dynamic_cast<FieldLine*>(other);

            // Distances vary depending on endpoint assignment.
            double d1 = (arma::norm(m_endPoints[0].groundCartesian - l->m_endPoints[0].groundCartesian, 2) + 
                            arma::norm(m_endPoints[1].groundCartesian - l->m_endPoints[1].groundCartesian, 2));
            double d2 = (arma::norm(m_endPoints[0].groundCartesian - l->m_endPoints[1].groundCartesian, 2) + 
                            arma::norm(m_endPoints[1].groundCartesian - l->m_endPoints[0].groundCartesian, 2));

            return std::min(d1, d2);
        }

        //double FieldLine::findError(const FieldLine& measured) const {
        //    return findError(Vector2<double>(measured.getScreenLineEquation().getRho(), measured.getScreenLineEquation().getPhi()));
        //}

        std::ostream& operator<< (std::ostream& output, const FieldLine& line) {
            output << "FieldLine " << std::endl;
            output << "Equation: " << line.m_screenLine << std::endl;
            output << "Field Equation: " << line.m_groundLine << std::endl;
            output << "\tpixelloc: [" << line.m_location.screenCartesian[0] << ", " << line.m_location.screenCartesian[1] << "]" << std::endl;
            output << " angularloc: [" << line.m_location.screenAngular[0] << ", " << line.m_location.screenAngular[1] << "]" << std::endl;
            output << "\trelative field coords: [" << line.m_location.neckRelativeRadialine[0] << ", " 
                                                    << line.m_location.neckRelativeRadial[1] << ", " 
                                                    << line.m_location.neckRelativeRadial[2] << "]" << std::endl;
            output << "\tspherical error: [" << line.m_sphericalError[0] << ", " << line.m_sphericalError[1] << "]" << std::endl;
            output << "\tsize on screen: [" << line.m_sizeOnScreen[0] << ", " << line.m_sizeOnScreen[1] << "]";

            return output;
        }

        std::ostream& operator<< (std::ostream& output, const std::vector<FieldLine>& lines) {
            for (const auto& : line : lines) {
                output << line << std::endl;
            }

            return output;
        }

        //void FieldLine::render(cv::Mat &mat, cv::Scalar colour) const {
        //    int width = mat.cols,
        //        height = mat.rows;
        //    Point p0(0,0), p1(width,0), p2(width,height), p3(0, height);
        //    Line l0(p0,p1), l1(p1,p2), l2(p2,p3), l3(p3, p0);

        //    Point left_i, right_i, top_i, bottom_i;

        //    bool left = m_screen_line.getIntersection(l3, left_i),
        //        right = m_screen_line.getIntersection(l1, right_i),
        //        top = m_screen_line.getIntersection(l0, top_i),
        //        bottom = m_screen_line.getIntersection(l2, bottom_i);

        //    Point render_pt1, render_pt2;
        //    if(left && left_i.y >= 0 && left_i.y <= height) {
        //        render_pt1 = left_i;
        //        if(top && top_i.x >= 0 && top_i.x <= width) {
        //            render_pt2 = top_i;
        //        }
        //        else if(bottom && bottom_i.x >=0 && bottom_i.x <= width) {
        //            render_pt2 = bottom_i;
        //        }
        //        else if(right && right_i.y >=0 && right_i.y <= height) {
        //            render_pt2 = right_i;
        //        }
        //        else {
        //            errorlog << "FieldLine::render - line outside image bounds" << std::endl;
        //            return;
        //        }
        //    }
        //    else if(right && right_i.y >=0 && right_i.y <= height){
        //        render_pt1 = right_i;
        //        if(top && top_i.x >= 0 && top_i.x <= width) {
        //            render_pt2 = top_i;
        //        }
        //        else if(bottom && bottom_i.x >=0 && bottom_i.x <= width) {
        //            render_pt2 = bottom_i;
        //        }
        //        else {
        //            errorlog << "FieldLine::render - line outside image bounds" << std::endl;
        //            return;
        //        }
        //    }
        //    else {
        //        if(top && bottom && top_i.x >= 0 && top_i.x <= width && bottom_i.x >=0 && bottom_i.x <= width) {
        //            render_pt1 = top_i;
        //            render_pt2 = bottom_i;
        //        }
        //        else {
        //            errorlog << "FieldLine::render - line outside image bounds" << std::endl;
        //            return;
        //        }
        //    }

        //    cv::line(mat, cv::Point2i(render_pt1.x, render_pt1.y), cv::Point2i(render_pt2.x, render_pt2.y), colour, 2);
        //}

    }
}
