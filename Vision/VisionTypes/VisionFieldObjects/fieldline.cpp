#include "fieldline.h"

FieldLine::FieldLine(const Line &line)
{
    m_id = FIELDLINE;
    
}


ostream& operator<< (ostream& output, const FieldLine& l)
{
    output << "FieldLine " << endl;
    output << "\tpixelloc: [" << l.m_location_pixels.x << ", " << l.m_location_pixels.y << "]" << endl;
    output << " angularloc: [" << l.m_location_angular.x << ", " << l.m_location_angular.y << "]" << endl;
    output << "\trelative field coords: [" << l.m_spherical_position.x << ", " << l.m_spherical_position.y << ", " << l.m_spherical_position.z << "]" << endl;
    output << "\ttransformed field coords: [" << l.m_transformed_spherical_pos.x << ", " << l.m_transformed_spherical_pos.y << ", " << l.m_transformed_spherical_pos.z << "]" << endl;
    output << "\tspherical error: [" << l.m_spherical_error.x << ", " << l.m_spherical_error.y << "]" << endl;
    output << "\tsize on screen: [" << l.m_size_on_screen.x << ", " << l.m_size_on_screen.y << "]";
    return output;
}

ostream& operator<< (ostream& output, const vector<FieldLine>& v)
{
    for (size_t i=0; i<v.size(); i++)
        output << v[i] << endl;
    return output;
}

void FieldLine::render(cv::Mat &mat) const
{
    int x_int = m_rho*cos(m_phi),
        y_int = m_rho*sin(m_phi);
    cv::line(mat, cv::Point2i(x_int, 0), cv::Point(0, y_int), cv::Scalar(0,0,255));
//    if(x_int > 0) {
//        if(y_int > 0) {

//        }
//        else {

//        }
//    }
//    else {
//        if(y_int > 0) {

//        }
//        else {
//            //line not on screen
//        }
//    }
}
