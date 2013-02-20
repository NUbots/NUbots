#include "point.h"

Point::Point(double screen_x, double screen_y) : screen(screen_x, screen_y) {}

Point::Point(Vector2<double> screen_pos) : screen(screen_pos) {}
