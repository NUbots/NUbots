/*
 * MIT License
 * ... (standard header)
 */
#include "SensorFilter.hpp"

#include <nlopt.hpp>
#include "utility/support/yaml_expression.hpp"

namespace module::input {

    using utility::support::Expression;

    // Template wrapper for NLopt (following FieldLocalisationNLopt pattern)
    template <typename Scalar, int N>
    using ObjectiveFunction = std::function<Scalar(const Eigen::Matrix<Scalar, N, 1>&, Eigen::Matrix<Scalar, N, 1>&, void*)>;

    template <typename Scalar, int N>
    Scalar eigen_objective_wrapper(const std::vector<Scalar>& x, std::vector<Scalar>& grad, void* data) {
        auto* obj_fun = static_cast<ObjectiveFunction<Scalar, N>*>(data);

        // Convert std::vector to Eigen
        Eigen::Matrix<Scalar, N, 1> eigen_x;
        if constexpr (N == Eigen::Dynamic) {
            eigen_x.resize(x.size());
            for (size_t i = 0; i < x.size(); ++i) {
                eigen_x[i] = x[i];
            }
        } else {
            for (int i = 0; i < N; ++i) {
                eigen_x[i] = x[i];
            }
        }

        Eigen::Matrix<Scalar, N, 1> eigen_grad;
        if constexpr (N == Eigen::Dynamic) {
            eigen_grad.resize(x.size());
        }

        Scalar result = (*obj_fun)(eigen_x, eigen_grad, nullptr);

        return result;
    }

    std::pair<Eigen::VectorXd, double> SensorFilter::optimize_sliding_window(
        const Eigen::VectorXd& initial_guess,
        const std::vector<MeasurementFactor>& factors) {

        // Store factors for objective function access
        current_factors_ = factors;

        // Create objective function
        ObjectiveFunction<double, Eigen::Dynamic> obj_fun =
            [this](const Eigen::VectorXd& x, Eigen::VectorXd& grad, void* data) -> double {
            return compute_sliding_window_cost(x);
        };

        // Setup NLopt
        const int n_vars = initial_guess.size();
        nlopt::algorithm algorithm = nlopt::LN_COBYLA;
        nlopt::opt opt = nlopt::opt(algorithm, n_vars);

        opt.set_xtol_rel(cfg.sliding_window.xtol_rel);
        opt.set_ftol_rel(cfg.sliding_window.ftol_rel);
        opt.set_maxeval(cfg.sliding_window.maxeval);

        opt.set_min_objective(eigen_objective_wrapper<double, Eigen::Dynamic>, &obj_fun);

        // Convert to std::vector
        std::vector<double> x(n_vars);
        for (int i = 0; i < n_vars; ++i) {
            x[i] = initial_guess[i];
        }

        // Optimize
        double final_cost;
        nlopt::result result = opt.optimize(x, final_cost);

        // Convert back
        Eigen::VectorXd solution(n_vars);
        for (int i = 0; i < n_vars; ++i) {
            solution[i] = x[i];
        }

        return {solution, final_cost};
    }

    double SensorFilter::compute_sliding_window_cost(const Eigen::VectorXd& states) {
        double total_cost = 0.0;

        // Measurement costs
        for (const auto& factor : current_factors_) {
            if (factor.time_index * 2 + 1 >= states.size()) continue;

            Eigen::Vector2d state_xy(states[factor.time_index * 2], states[factor.time_index * 2 + 1]);
            Eigen::Vector2d residual = factor.measurement - state_xy;
            total_cost += factor.weight * residual.squaredNorm();
        }

        // Smoothness regularization - normalize by window size to make weight more intuitive
        int window_size = states.size() / 2;
        if (window_size > 1) {
            // Scale smoothness weight by number of constraints to make it comparable to measurement weights
            // double normalized_smoothness_weight = cfg.sliding_window.smoothness_weight / (window_size - 1);

            for (int k = 0; k < window_size - 1; ++k) {
                Eigen::Vector2d state_k(states[2*k], states[2*k + 1]);
                Eigen::Vector2d state_k1(states[2*(k+1)], states[2*(k+1) + 1]);
                Eigen::Vector2d diff = state_k1 - state_k;
                total_cost += cfg.sliding_window.smoothness_weight * diff.squaredNorm();
            }
        }

        return total_cost;
    }

}  // namespace module::input
