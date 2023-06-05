#include "OccupancyMap.hpp"

#include <Eigen/Core>

namespace module::localisation {

    Eigen::MatrixXd OccupancyMap::get_map() {
        return map;
    }

    int OccupancyMap::get_width() {
        return map.cols();
    }

    int OccupancyMap::get_length() {
        return map.rows();
    }

    double OccupancyMap::get_occupancy_value(int x, int y) {
        // Check if the observation is within the map
        if (x < 0 || x >= get_length() || y < 0 || y >= get_width()) {
            return -1;
        }
        else {
            return map(x, y);
        }
    }

    void OccupancyMap::resize(int width, int length) {
        map.resize(width, length);
        map.setZero();
    }

    void OccupancyMap::add_horizontal_line(int x0, int y0, int length) {
        map.block(y0, x0, 1, length).setOnes();
    }

    void OccupancyMap::add_vertical_line(int x0, int y0, int length) {
        map.block(y0, x0, length, 1).setOnes();
    }
    void OccupancyMap::add_rectangle(int x0, int y0, int width, int height) {
        // Add the top and bottom lines
        add_horizontal_line(x0, y0, width);
        add_horizontal_line(x0, y0 + height - 1, width);
        // Add the left and right lines
        add_vertical_line(x0, y0, height);
        add_vertical_line(x0 + width - 1, y0, height);
    }

    void OccupancyMap::add_rectangle(int x0, int y0, int width, int height, int inner_width) {
        for (int i = 0; i < inner_width; i++) {
            add_rectangle(x0 + i, y0 + i, width - (2 * i), height - (2 * i));
        }
    }

    void OccupancyMap::add_circle(const int X0, const int Y0, const int radius, const int inner_width) {
        for (int y = Y0 - radius; y <= Y0 + radius; y++) {
            for (int x = X0 - radius; x <= X0 + radius; x++) {
                if (std::pow(x - X0, 2) + std::pow(y - Y0, 2) <= std::pow(radius, 2)) {
                    int dist = std::sqrt(std::pow(x - X0, 2) + std::pow(y - Y0, 2));
                    if (dist >= radius - inner_width && dist <= radius) {
                        if (x >= 0 && x < map.cols() && y >= 0 && y < map.rows()) {
                            map(y, x) = 1;
                        }
                    }
                }
            }
        }
    }

    void OccupancyMap::add_cross(const int X0, const int Y0, const int radius, const int inner_width) {
        for (int i = Y0 - radius; i <= Y0 + radius; i++) {
            if (i >= 0 && i < map.rows()) {
                int width   = inner_width / 2;
                int start_x = X0 - width;
                int end_x   = X0 + width;
                if (start_x >= 0 && end_x < map.cols()) {
                    map.block(i, start_x, 1, end_x - start_x + 1) = Eigen::MatrixXd::Ones(1, end_x - start_x + 1);
                }
            }
        }

        for (int i = X0 - radius; i <= X0 + radius; i++) {
            if (i >= 0 && i < map.cols()) {
                int width   = inner_width / 2;
                int start_y = Y0 - width;
                int end_y   = Y0 + width;
                if (start_y >= 0 && end_y < map.rows()) {
                    map.block(start_y, i, end_y - start_y + 1, 1) = Eigen::MatrixXd::Ones(end_y - start_y + 1, 1);
                }
            }
        }
    }

    void OccupancyMap::fill_surrounding_cells(int range) {
        int rows = map.rows();
        int cols = map.cols();
        for (int y = 0; y < rows; y++) {
            for (int x = 0; x < cols; x++) {
                if (map(y, x) == 1) {
                    for (int i = 1; i <= range; i++) {
                        double value = (1.0 * (range - i) / range) * 0.5;
                        for (int j = -i; j <= i; j++) {
                            if (x - i >= 0 && y + j >= 0 && y + j < rows) {
                                if (map(y + j, x - i) < value) {
                                    map(y + j, x - i) = value;
                                }
                            }
                            if (x + i < cols && y + j >= 0 && y + j < rows) {
                                if (map(y + j, x + i) < value) {
                                    map(y + j, x + i) = value;
                                }
                            }
                            if (y - i >= 0 && x + j >= 0 && x + j < cols) {
                                if (map(y - i, x + j) < value) {
                                    map(y - i, x + j) = value;
                                }
                            }
                            if (y + i < rows && x + j >= 0 && x + j < cols) {
                                if (map(y + i, x + j) < value) {
                                    map(y + i, x + j) = value;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    void OccupancyMap::create_distance_map(double grid_size) {
        // Initialize the distance map with infinite distances
        Eigen::MatrixXd dist_map =
            Eigen::MatrixXd::Constant(map.rows(), map.cols(), std::numeric_limits<double>::infinity());

        // Create a queue to hold the cells to be visited
        std::queue<std::pair<int, int>> q;

        // Mark all occupied cells as visited and set their distances to 0
        for (int y = 0; y < map.rows(); y++) {
            for (int x = 0; x < map.cols(); x++) {
                if (map(y, x) == 1) {
                    dist_map(y, x) = 0;
                    q.push(std::make_pair(y, x));
                }
            }
        }

        // Run BFS algorithm to compute the distances of all unoccupied cells to the nearest occupied cell
        while (!q.empty()) {
            std::pair<int, int> curr = q.front();
            q.pop();
            int y = curr.first;
            int x = curr.second;

            // Update the distances of the neighbors of the current cell
            for (int j = -1; j <= 1; j++) {
                for (int i = -1; i <= 1; i++) {
                    // Skip the current cell
                    if (i == 0 && j == 0) {
                        continue;
                    }

                    int new_y = y + j;
                    int new_x = x + i;

                    // Check if the neighbor is within the map boundaries
                    if (new_y >= 0 && new_y < map.rows() && new_x >= 0 && new_x < map.cols()) {
                        double weight = std::sqrt(std::pow(i, 2) + std::pow(j, 2));

                        // Check if the neighbor is unoccupied and has a shorter distance through the current cell
                        if (map(new_y, new_x) == 0 && dist_map(new_y, new_x) > dist_map(y, x) + weight) {
                            dist_map(new_y, new_x) = dist_map(y, x) + weight;
                            q.push(std::make_pair(new_y, new_x));
                        }
                    }
                }
            }
        }

        // Replace the original map with the distance map
        map = dist_map * grid_size;
    }


}  // namespace module::localisation
