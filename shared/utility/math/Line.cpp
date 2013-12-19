/*
 * This file is part of NUBots Utility.
 *
 * NUBots Utility is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * NUBots Utility is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with NUBots Utility.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "Line.h"

namespace utility {
    namespace math {

        // Constructor
        Line::Line() : v(arma::zeros<arma::vec>(2)), a(arma::zeros<arma::vec>(2)) {
            // General Line Equation: A*x + B*y = C
            m_A = 0.0;
            m_B = 0.0;
            m_C = 0.0;
            m_rho = 0.0;
            m_phi = 0.0;
            m_inv_normaliser = 1.0;
        }

        // Constructor
        Line::Line(const arma::vec2& p1, const arma::vec2& p2) {
            // General Line Equation: A*x + B*y = C
            setLineFromPoints(p1, p2);
        }

        Line::Line(double rho, double phi) {
            setLine(rho, phi);
        }

        // Destructor
        Line::~Line() {
            return;
        }

        // setLine(float A, float B, float C): Set the line to the given A, B, C values, if they are valid.
        bool Line::setLine(double A, double B, double C) {
            // If the values do not give a valid line, do not use them.
            if (isValid(A, B, C) == false) {
              return false;
            }

            if (B != 0.0) {
                // B != 0 means not vertical
                m_A = A / B;
                m_B = 1;
                m_C = C / B;
                m_inv_normaliser = 1.0 / sqrt((m_A * m_A) + (m_B * m_B));
                m_phi = acos(m_A * m_inv_normaliser);
                m_rho = m_C * m_inv_normaliser;
                a << 0 << m_C;
            }

            else {
                // B == 0 means vertical
                m_A = 1;
                m_B = 0.0;
                m_C = C / A;
                m_phi = 0.0;
                m_rho = m_C;
                m_inv_normaliser = 1.0 / m_A;
                a << m_C << 0;
            }

            v << m_B << -m_A;
            v = v / arma::norm(v, 2);

            normaliseRhoPhi();

            // Just to double check.
            return (isValid());
        }

        bool Line::setLine(double rho, double phi) {
            m_rho = rho;
            m_phi = phi;
            m_A = cos(phi);
            m_B = sin(phi);
            m_C = rho;
            m_inv_normaliser = 1.0 / sqrt((m_A * m_A) + (m_B * m_B));

            if (m_B == 0) {
                a << m_C << 0;
            }

            else {
                a << 0 << m_C;
            }

            v  << m_B << -m_A;
            v = v / arma::norm(v, 2);

            normaliseRhoPhi();

            // Lines in this form are always valid.
            return true;
        }

        void Line::normaliseRhoPhi() {
            m_phi = m_phi - 2 * datum::pi * floor(m_phi / (2 * arma::pi));
        }

        // setLineFromPoints(arma::vec2 p1, arma::vec2 p2): Generate the line that passes through the two given points.
        bool Line::setLineFromPoints(const arma::vec2& p1, const arma::vec2& p2) {
            // Using method found at: http://www.uwm.edu/~ericskey/TANOTES/Ageometry/node4.html
            double A, B, C;
            double xDiff, yDiff;

            // Cannot make a line between 2 points if they are the same point.
            if ((p1[0] == p2[0]) && (p1[1] == p2[1])) {
                return false;
            }

            // Go from substitution: p1[0]*A + p1[1]*B = p2[0]*A + p2[1]*B --> xDIff*A = yDiff*B (1).
            xDiff = p1[0] - p2[0];
            yDiff = p2[1] - p1[1];

            // Using enforced relationship A^2 + B^2 = 1 and previous result, find A.
            A = (yDiff * yDiff) / ((yDiff * yDiff) + (xDiff * xDiff));

            // If A = 0.0 equation is of the form y = C, so B = 1.0 stopping divide by zero. Otherwise calculate from expression (1)
            if (A == 0.0) {
                B = 1.0;
            }

            else {
                B = (xDiff * A) / yDiff;
            }

            // Substitute first point to find C.
            C = (A * p1[0]) + (B * p1[1]);
            
            return setLine(A, B, C); // Now try to set the values.
        }

        bool Line::copy(const Line& source) {
            return setLine(source.getA(), source.getB(), source.getC());
        }

        double Line::getA() const {
            return m_A;
        }

        double Line::getB() const {
            return m_B;
        }

        double Line::getC() const {
            return m_C;
        }

        // isHorizontal(): Check if the current line is purely horizontal i.e. of the form y = C
        bool Line::isHorizontal() const {
            return (m_A == 0.0);
        }

        // isHorizontal(): Check if the current line is purely vertical i.e. of the form x = C
        bool Line::isVertical() const {
            return (m_B == 0.0);
        }

        // isValid(): Check if the current line is valid.
        bool Line::isValid() const {
            return isValid(m_A, m_B, m_C);
        }

        // findYFromX(float x): Calculate the x coord given the y for the current line.
        double Line::findXFromY(double y) const {
            double x = std::numeric_limits<double>::max();

            // If horizontal cannot find x from y.
            if ((isValid() == true) && (isHorizontal() == false)) {
                // rearrange --> x = (C - B * y) / A
                x = (m_C - m_B * y) / m_A;
            }

            return x;
        }

        // findYFromX(float x): Calculate the y coord given the x for the current line.
        double Line::findYFromX(double x) const {
            double y = std::numeric_limits<double>::max();

            // If vertical cannot find y from x.
            if ((isValid() == true) && (isVertical() == false)) {
               // rearrange --> y = (C - A * x) / B
               y = (m_C - m_A * x) / m_B;
            }

            return y;
        }

        // getGradient(): Return the gradient of the current line.
        double Line::getGradient() const {
            double gradient;

            if (!isValid()) {
                gradient = 0.0;
            }

            // Big number to represent infinity.
            else if (isVertical()) {
                gradient = std::numeric_limits<double>::max();
            }

            // Rearrange equation --> y = C/B - A/B*x
            else {
                gradient = -(m_A / m_B);
            }

            return gradient;
        }

        double Line::getAngle() const {
          return atan(getGradient());
        }

        double Line::getXIntercept() const {
          return findXFromY(0);
        }

        double Line::getYIntercept() const {
          return findYFromX(0);
        }

        double Line::getLinePointDistance(const arma::vec2& point) const {
            return std::abs((m_A * point[0]) + (m_B * point[1]) - m_C) * m_inv_normaliser;
        }

        double Line::getSignedLinePointDistance(const arma::vec2& point) const {
          return ((m_A * point[0]) + (m_B * point[1]) - m_C) * m_inv_normaliser;
        }

        double Line::getAngleBetween(const Line& other) const {
            double angle = std::abs(getAngle() - other.getAngle());

            if (angle > (datum::pi * 0.5))
                angle = datum::pi - angle;

            return angle;
        }

        double Line::getRho() const {
            return m_rho;
        }

        double Line::getPhi() const {
            return m_phi;
        }

        double Line::scalarProjection(const arma::vec2& point) const {
            return arma::dot(point - a,  v);
        }

        arma::vec2 Line::projectOnto(const arma::vec2& point) const {
            return (scalarProjection(point) * v + a);
        }

        std::vector< arma::vec2 > Line::projectOnto(const std::vector<arma::vec2>& points) const {
            std::vector<arma::vec2> result;

            for (const arma::vec2& point : points) {
                result.push_back(projectOnto(point));
            }

            return result;
        }

        bool Line::getIntersection(const Line& other, arma::vec2& point) const {
            // (this)  A_1*x + B_1*y = C_1 ==> y = (C_1 / B_1) - (A_1 / B_1) * x
            // (other) A_2*x + B_2*y = C_2 ==> y = (C_2 / B_2) - (A_2 / B_2) * x

            // At the point of intersection, y values are equal.
            // (C_1 / B_1) - (A_1 / B_1) * x = (C_1 / B_1) - (A_1 / B_1) * x

            // Sove for x.
            // x = ((B_2 * C_1) - (B_1 * C_2)) / ((A_1 * B_2) - (A_2 * B_1))

            // Then
            // y = (C_1 / B_1) - (A_1 / B_1) * (((B_2 * C_1) - (B_1 * C_2)) / ((A_1 * B_2) - (A_2 * B_1)))
            //   = ((A_1 * B_2 * C_1) - (A_2 * B_1 * C_1) - (A_1 * B_2 * C_1) + (A_1 * B_1 * C_2)) / (((A_1 * B_2) - (A_2 * B_1)) * B_1)
            //   = (((A_1 * C_2) - (A_2 * C_1)) * B_1) / (((A_1 * B_2) - (A_2 * B_1)) * B_1)
            //   = ((A_1 * C_2) - (A_2 * C_1)) / ((A_1 * B_2) - (A_2 * B_1))
            double norm = ((m_A * other.getB()) - (m_B * other.getA()));

            // If norm is 0 then lines don't intersect.
            if (norm != 0) {
                point[0] = ((m_C * other.getB()) - (m_B * other.getC())) / norm;
                point[1] = ((m_A * other.getC()) - (m_C * other.getA())) / norm;

                return true;
            }

            // No intersection.
            return false;
        }

        bool operator == (const Line& line1, const Line& line2) {
            return ((line1.getA() == line2.getA()) && (line1.getB() == line2.getB()) && (line1.getC() == line2.getC()));
        }

        bool operator != (const Line& line1, const Line& line2) {
          return !(line1 == line2);
        }

        bool operator > (const Line& line1, const Line& line2) {
            if(line1.getGradient() == line2.getGradient()) {
                return (line1.getYIntercept() > line2.getYIntercept());
            }

            else {
                return (line1.getGradient() > line2.getGradient());
            }
        }

        bool operator < (const arma::vec2& point1, const arma::vec2& point2) {
            if(point1[0] < point2[0]) {
                return true;
            }

            else if(point1[0] == point2[0]) {
                if(point1[1] < point2[1]) {
                    return true;
                }
            }

            return false;
        }

        std::ostream& operator<< (std::ostream& output, const Line& line) {
            output << line.getA() << "x + " << line.getB() << "y = " << line.getC();

            return output;
        }

        // isValid(float A, float B, float C): Check if the given values create a valid line.
        bool Line::isValid(double A, double B, double C) const {
            (void)(C); // To stop compiler warnings.

            return ((A != 0.0) || (B != 0.0)); // If A = 0.0 and B = 0.0 line is not valid, as equation becomes 0.0 = C
        }

    }
}
        
