#ifndef LOCALISATION_H
#define LOCALISATION_H

#include <Eigen/Core>


struct LocalisationParameters {
    // 2 vectors of opponent goal post (opponent, left and right)
    std::vector<Eigen::Vector2d> goal_posts = {Eigen::Vector2d(4.5, 1.3), Eigen::Vector2d(4.5, -1.3)};
    // Process model noise, diagonal matrix
    Eigen::MatrixXd process_noise = Eigen::MatrixXd::Zero(2, 2);
    // Measurement model noise, diagonal
    Eigen::MatrixXd measurement_noise = Eigen::MatrixXd::Zero(2, 2);
};

struct LocalisationProcessModel {
    void operator()(const Eigen::VectorXd& x,
                    const Eigen::VectorXd& u,
                    const LocalisationParameters& param,
                    Eigen::VectorXd& f);
    void operator()(const Eigen::VectorXd& x,
                    const Eigen::VectorXd& u,
                    const LocalisationParameters& param,
                    Eigen::VectorXd& f,
                    Eigen::MatrixXd& SQ);
    void operator()(const Eigen::VectorXd& x,
                    const Eigen::VectorXd& u,
                    const LocalisationParameters& param,
                    Eigen::VectorXd& f,
                    Eigen::MatrixXd& SQ,
                    Eigen::MatrixXd& dfdx);
};

struct LocalisationMeasurementModel {
    void operator()(const Eigen::VectorXd& x,
                    const Eigen::VectorXd& u,
                    const LocalisationParameters& param,
                    Eigen::VectorXd& h);
    void operator()(const Eigen::VectorXd& x,
                    const Eigen::VectorXd& u,
                    const LocalisationParameters& param,
                    Eigen::VectorXd& h,
                    Eigen::MatrixXd& SR);
    void operator()(const Eigen::VectorXd& x,
                    const Eigen::VectorXd& u,
                    const LocalisationParameters& param,
                    Eigen::VectorXd& h,
                    Eigen::MatrixXd& SR,
                    Eigen::MatrixXd& dhdx);
};

// struct LocalisationLogLikelihood {
//     double operator()(const Eigen::VectorXd& y,
//                       const Eigen::VectorXd& x,
//                       const Eigen::VectorXd& u,
//                       const LocalisationParameters& param);
//     double operator()(const Eigen::VectorXd& y,
//                       const Eigen::VectorXd& x,
//                       const Eigen::VectorXd& u,
//                       const LocalisationParameters& param,
//                       Eigen::VectorXd& g);
//     double operator()(const Eigen::VectorXd& y,
//                       const Eigen::VectorXd& x,
//                       const Eigen::VectorXd& u,
//                       const LocalisationParameters& param,
//                       Eigen::VectorXd& g,
//                       Eigen::MatrixXd& H);
// };

#endif
