#include "fieldline.h"
#include "debug.h"

FieldLine::FieldLine(const Line &line)
{
    set(line.getRho(), line.getPhi());
}

FieldLine::FieldLine(double rho, double phi)
{
    set(rho, phi);
}

void FieldLine::set(double rho, double phi)
{
    m_id = FIELDLINE;
    m_rho = rho;
    m_phi = phi;
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
    int width = mat.cols,
        height = mat.rows;
    Point p0(0,0), p1(width,0), p2(width,height), p3(0, height);
    Line l0(p0,p1), l1(p1,p2), l2(p2,p3), l3(p3, p0);
    Line line(m_rho, m_phi);

    Point left_i, right_i, top_i, bottom_i;

    bool left = line.getIntersection(l3, left_i),
        right = line.getIntersection(l1, right_i),
        top = line.getIntersection(l0, top_i),
        bottom = line.getIntersection(l2, bottom_i);

    Point render_pt1, render_pt2;
    if(left && left_i.y >= 0 && left_i.y <= height) {
        render_pt1 = left_i;
        if(top && top_i.x >= 0 && top_i.x <= width) {
            render_pt2 = top_i;
        }
        else if(bottom && bottom_i.x >=0 && bottom_i.x <= width) {
            render_pt2 = bottom_i;
        }
        else if(right && right_i.y >=0 && right_i.y <= height) {
            render_pt2 = right_i;
        }
        else {
            errorlog << "FieldLine::render - line outside image bounds" << endl;
            return;
        }
    }
    else if(right && right_i.y >=0 && right_i.y <= height){
        render_pt1 = right_i;
        if(top && top_i.x >= 0 && top_i.x <= width) {
            render_pt2 = top_i;
        }
        else if(bottom && bottom_i.x >=0 && bottom_i.x <= width) {
            render_pt2 = bottom_i;
        }
        else {
            errorlog << "FieldLine::render - line outside image bounds" << endl;
            return;
        }
    }
    else {
        if(top && bottom && top_i.x >= 0 && top_i.x <= width && bottom_i.x >=0 && bottom_i.x <= width) {
            render_pt1 = top_i;
            render_pt2 = bottom_i;
        }
        else {
            errorlog << "FieldLine::render - line outside image bounds" << endl;
            return;
        }
    }

    cv::line(mat, cv::Point2i(render_pt1.x, render_pt1.y), cv::Point2i(render_pt2.x, render_pt2.y), cv::Scalar(0,0,255));
}
