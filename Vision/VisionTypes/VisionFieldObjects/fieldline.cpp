#include "fieldline.h"
#include "debug.h"

FieldLine::FieldLine(const LSFittedLine &screen_line, const LSFittedLine &relative_line)
{
    m_id = FIELDLINE;
    set(screen_line, relative_line);
}

FieldLine::FieldLine(const Vector2<NUPoint>& end_points)
{
    m_id = FIELDLINE;
    set(end_points);
}

void FieldLine::set(const LSFittedLine& screen_line, const LSFittedLine& ground_line)
{
    m_screen_line = screen_line;
    m_ground_line = ground_line;

    if(screen_line.valid) {
        screen_line.getEndPoints(m_end_points[0].screenCartesian, m_end_points[1].screenCartesian);
    }
    else {
        m_end_points[0].screenCartesian = Vector2<double>(-1,-1);
        m_end_points[1].screenCartesian = Vector2<double>(-1,-1);   //-1 is an invalid pixel location
    }

    if(ground_line.valid) {
        ground_line.getEndPoints(m_end_points[0].groundCartesian, m_end_points[1].groundCartesian);
    }
    else{
        m_end_points[0].groundCartesian = Vector2<double>(-1,-1);
        m_end_points[1].groundCartesian = Vector2<double>(-1,-1);   //-1 is an impossible ground location
    }
}

void FieldLine::set(const Vector2<NUPoint>& end_points)
{
    m_screen_line.setLineFromPoints(end_points.x.screenCartesian, end_points.y.screenCartesian);
    m_ground_line.setLineFromPoints(end_points.x.groundCartesian, end_points.y.groundCartesian);
    m_end_points = end_points;
}

void FieldLine::printLabel(ostream &out) const
{
    out << m_end_points;
}

double FieldLine::findScreenError(VisionFieldObject* other) const
{
    FieldLine* l = dynamic_cast<FieldLine*>(other);

    // distances vary depending on endpoint assignment
    double d1 = ( m_end_points[0].screenCartesian - l->m_end_points[0].screenCartesian ).abs() + ( m_end_points[1].screenCartesian - l->m_end_points[1].screenCartesian ).abs();
    double d2 = ( m_end_points[0].screenCartesian - l->m_end_points[1].screenCartesian ).abs() + ( m_end_points[1].screenCartesian - l->m_end_points[0].screenCartesian ).abs();

    return min(d1, d2);
}

double FieldLine::findGroundError(VisionFieldObject* other) const
{
    FieldLine* l = dynamic_cast<FieldLine*>(other);

    // distances vary depending on endpoint assignment
    double d1 = ( m_end_points[0].groundCartesian - l->m_end_points[0].groundCartesian ).abs() + ( m_end_points[1].groundCartesian - l->m_end_points[1].groundCartesian ).abs();
    double d2 = ( m_end_points[0].groundCartesian - l->m_end_points[1].groundCartesian ).abs() + ( m_end_points[1].groundCartesian - l->m_end_points[0].groundCartesian ).abs();

    return min(d1, d2);
}

//double FieldLine::findError(const FieldLine& measured) const
//{
//    return findError(Vector2<double>(measured.getScreenLineEquation().getRho(), measured.getScreenLineEquation().getPhi()));
//}

ostream& operator<< (ostream& output, const FieldLine& l)
{
    output << "FieldLine " << endl;
    output << "Equation: " << l.m_screen_line << endl;
    output << "Field Equation: " << l.m_ground_line << endl;
    output << "\tpixelloc: [" << l.m_location.screenCartesian.x << ", " << l.m_location.screenCartesian.y << "]" << endl;
    output << " angularloc: [" << l.m_location.screenAngular.x << ", " << l.m_location.screenAngular.y << "]" << endl;
    output << "\trelative field coords: [" << l.m_location.neckRelativeRadial.x << ", " << l.m_location.neckRelativeRadial.y << ", " << l.m_location.neckRelativeRadial.z << "]" << endl;
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

//void FieldLine::render(cv::Mat &mat, cv::Scalar colour) const
//{
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
//            errorlog << "FieldLine::render - line outside image bounds" << endl;
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
//            errorlog << "FieldLine::render - line outside image bounds" << endl;
//            return;
//        }
//    }
//    else {
//        if(top && bottom && top_i.x >= 0 && top_i.x <= width && bottom_i.x >=0 && bottom_i.x <= width) {
//            render_pt1 = top_i;
//            render_pt2 = bottom_i;
//        }
//        else {
//            errorlog << "FieldLine::render - line outside image bounds" << endl;
//            return;
//        }
//    }

//    cv::line(mat, cv::Point2i(render_pt1.x, render_pt1.y), cv::Point2i(render_pt2.x, render_pt2.y), colour, 2);
//}
