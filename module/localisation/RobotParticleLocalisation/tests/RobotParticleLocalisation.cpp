// Uncomment this line when other test files are added
//#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
//#include <catch.hpp>

// Remove this line when test files are added
int main() {

    // std::vector<Eigen::Vector3d> position_readings{};
    // std::vector<Eigen::Matrix3d> covariance_readings{};
    // std::vector<Eigen::Matrix4d> Hcw_readings{};
    // std::vector<int> side_readings{};
    // std::vector<Eigen::Vector3d> truth_readings{};

    // char comma;

    // std::ifstream ifs("tests/left_field_static.csv");
    // while (ifs.good()) {
    //     Eigen::Vector3d position;
    //     Eigen::Matrix3d covariance;
    //     Eigen::Matrix4d Hcw;
    //     int side;

    //     // Hardcoded start state
    //     Eigen::Vector3d truth(-3.5, 3.31, 1.57);

    //     ifs >> position(0) >> comma >> position(1) >> comma >> position(2) >> cov(0, 0) >> comma >> cov(0, 1) >>
    //     comma
    //         >> cov(0, 2) >> comma >> cov(1, 0) >> comma >> cov(1, 1) >> comma >> cov(1, 2) >> comma >> cov(2, 0)
    //         >> comma >> cov(2, 1) >> comma >> cov(2, 2) >> comma >> Hcw(0, 0) >> comma >> Hcw(0, 1) >> comma
    //         >> Hcw(0, 2) >> comma >> Hcw(0, 3) >> comma >> Hcw(1, 0) >> comma >> Hcw(1, 1) >> comma >> Hcw(1, 2)
    //         >> comma >> Hcw(1, 3) >> comma >> Hcw(2, 0) >> comma >> Hcw(2, 1) >> comma >> Hcw(2, 2) >> comma
    //         >> Hcw(2, 3) >> comma >> Hcw(3, 0) >> comma >> Hcw(3, 1) >> comma >> Hcw(3, 2) >> comma >> Hcw(3, 3)
    //         >> comma >> goal_post.side;


    //     if (ifs.good()) {
    //         position_readings.emplace_bacK(position);
    //         covariance_readings.emplace_bacK(covariance);
    //         Hcw_readings.emplace_bacK(Hcw);
    //         side_readings.emplace_bacK(side);
    //         truth_readings.emplace_back(truth);
    //     }
    // }
    // ifs.close();
    return 0;
}
