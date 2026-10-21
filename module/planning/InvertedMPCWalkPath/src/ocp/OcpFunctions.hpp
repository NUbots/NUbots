/*
 * MIT License
 *
 * Copyright (c) 2026 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef MODULE_PLANNING_INVERTEDMPCWALKPATH_OCP_OCPFUNCTIONS_HPP
#define MODULE_PLANNING_INVERTEDMPCWALKPATH_OCP_OCPFUNCTIONS_HPP

#include <Eigen/Core>
#include <array>
#include <autodiff/forward/dual.hpp>
#include <autodiff/forward/dual/eigen.hpp>
#include <cstddef>

namespace module::planning::inverted_walk_mpc::ocp {

    /// @brief One of the functions acados evaluates, in the C calling convention of a CasADi-generated function,
    /// which acados's external_function_external_param_casadi wraps: the function, its workspace size, the sparsity
    /// of each input and output, and how many there are. The parameter vector is the last input.
    struct ExternalFunction {
        int (*fun)(const double**, double**, int*, double*, void*) = nullptr;
        int (*work)(int*, int*, int*, int*)                        = nullptr;
        const int* (*sparsity_in)(int)                             = nullptr;
        const int* (*sparsity_out)(int)                            = nullptr;
        int (*n_in)()                                              = nullptr;
        int (*n_out)()                                             = nullptr;
    };

    /// @brief The value, Jacobian and Hessian functions of one part of the problem, as acados asks for them
    struct FunctionSet {
        ExternalFunction value{};
        ExternalFunction jacobian{};
        ExternalFunction hessian{};
    };

    /// @brief Every function of an optimal control problem with discrete dynamics, external costs and nonlinear
    /// constraints
    struct Functions {
        FunctionSet dynamics{};
        FunctionSet cost{};
        FunctionSet terminal_cost{};
        FunctionSet constraint{};
        FunctionSet terminal_constraint{};
    };

    /// @brief A dense matrix argument in CasADi's compact sparsity format: rows, columns, 1 for dense
    using Shape = std::array<int, 3>;
    constexpr Shape dense(const int rows, const int cols = 1) {
        return {rows, cols, 1};
    }

    /// @brief The shapes of a function's inputs and outputs
    template <std::size_t I, std::size_t O>
    struct Signature {
        std::array<Shape, I> in;
        std::array<Shape, O> out;
    };

    /// @brief Exposes Evaluate, which reads dense column-major inputs and writes dense column-major outputs, as an
    /// ExternalFunction with signature S
    template <auto S, void (*Evaluate)(const double**, double**)>
    ExternalFunction external_function() {
        struct Abi {
            static int fun(const double** arg, double** res, int* /*iw*/, double* /*w*/, void* /*mem*/) {
                Evaluate(arg, res);
                return 0;
            }
            static int work(int* sz_arg, int* sz_res, int* sz_iw, int* sz_w) {
                *sz_arg = int(S.in.size());
                *sz_res = int(S.out.size());
                *sz_iw  = 0;
                *sz_w   = 0;
                return 0;
            }
            static const int* sparsity_in(const int i) {
                return S.in[i].data();
            }
            static const int* sparsity_out(const int i) {
                return S.out[i].data();
            }
            static int n_in() {
                return int(S.in.size());
            }
            static int n_out() {
                return int(S.out.size());
            }
        };
        return {&Abi::fun, &Abi::work, &Abi::sparsity_in, &Abi::sparsity_out, &Abi::n_in, &Abi::n_out};
    }

    /// @brief The value, gradient and Hessian of a scalar function f(z), f being generic in its scalar type
    template <typename Fun>
    void scalar_derivatives(const Fun& f, const Eigen::VectorXd& z0, double& value, double* gradient, double* hessian) {
        autodiff::VectorXdual2nd z = z0.cast<autodiff::dual2nd>();
        autodiff::dual2nd u;
        Eigen::VectorXd g;
        const Eigen::MatrixXd H = autodiff::hessian(f, autodiff::wrt(z), autodiff::at(z), u, g);
        value                   = autodiff::val(u);
        if (gradient != nullptr) {
            Eigen::Map<Eigen::VectorXd>(gradient, g.size()) = g;
        }
        if (hessian != nullptr) {
            Eigen::Map<Eigen::MatrixXd>(hessian, H.rows(), H.cols()) = H;
        }
    }

    /// @brief The value and gradient of a scalar function f(z), f being generic in its scalar type
    template <typename Fun>
    void scalar_gradient(const Fun& f, const Eigen::VectorXd& z0, double& value, double* gradient) {
        autodiff::VectorXdual z = z0.cast<autodiff::dual>();
        autodiff::dual u;
        Eigen::VectorXd g;
        autodiff::gradient(f, autodiff::wrt(z), autodiff::at(z), u, g);
        value                                           = autodiff::val(u);
        Eigen::Map<Eigen::VectorXd>(gradient, g.size()) = g;
    }

    /// @brief The value and transposed Jacobian of a vector function f(z), f being generic in its scalar type
    template <typename Fun>
    void vector_jacobian(const Fun& f, const Eigen::VectorXd& z0, double* value, double* jacobian_transposed) {
        autodiff::VectorXdual z = z0.cast<autodiff::dual>();
        autodiff::VectorXdual F;
        const Eigen::MatrixXd J = autodiff::jacobian(f, autodiff::wrt(z), autodiff::at(z), F);
        for (int i = 0; i < F.size(); ++i) {
            value[i] = autodiff::val(F[i]);
        }
        Eigen::Map<Eigen::MatrixXd>(jacobian_transposed, J.cols(), J.rows()) = J.transpose();
    }

    /// @brief acados's functions for a problem P, with
    ///   P::NX, P::NU, P::NP, P::NH                         state, input, parameter and constraint sizes
    ///   P::dynamics<T>(x, u, p) -> x⁺                       discrete dynamics
    ///   P::cost<T>(x, u, p), P::terminal_cost<T>(x, p)      stage and terminal costs
    ///   P::constraint<T>(x, u, p), P::terminal_constraint<T>(x, p) -> h   nonlinear constraints h ≤ uh
    /// each generic in its scalar type T so autodiff can differentiate it. acados orders the decision variables of a
    /// stage as z = (u, x), and wants the Jacobians transposed.
    template <typename P>
    struct ProblemFunctions {
        static constexpr int NX  = P::NX;
        static constexpr int NU  = P::NU;
        static constexpr int NP  = P::NP;
        static constexpr int NH  = P::NH;
        static constexpr int NUX = NU + NX;

        template <typename T>
        using State = Eigen::Matrix<T, NX, 1>;
        template <typename T>
        using Input = Eigen::Matrix<T, NU, 1>;

        /// @brief z = (u, x)
        static Eigen::VectorXd stage(const double* x, const double* u) {
            Eigen::VectorXd z(NUX);
            z << Eigen::Map<const Eigen::VectorXd>(u, NU), Eigen::Map<const Eigen::VectorXd>(x, NX);
            return z;
        }

        template <typename Z>
        static auto input(const Z& z) {
            return Input<typename Z::Scalar>(z.template head<NU>());
        }
        template <typename Z>
        static auto state(const Z& z) {
            return State<typename Z::Scalar>(z.template tail<NX>());
        }

        // Dynamics: inputs (x, u, p), or (x, u, λ, p) for the Hessian of λᵀx⁺
        static void dynamics(const double** in, double** out) {
            const Eigen::VectorXd z                 = stage(in[0], in[1]);
            Eigen::Map<Eigen::VectorXd>(out[0], NX) = P::template dynamics<double>(state(z), input(z), in[2]);
        }
        static void dynamics_jacobian(const double** in, double** out) {
            const double* p = in[2];
            vector_jacobian(
                [p](const auto& z) {
                    using T = typename std::decay_t<decltype(z)>::Scalar;
                    return Eigen::Matrix<T, Eigen::Dynamic, 1>(P::template dynamics<T>(state(z), input(z), p));
                },
                stage(in[0], in[1]),
                out[0],
                out[1]);
        }
        static void dynamics_hessian(const double** in, double** out) {
            const double* p = in[3];
            const Eigen::Map<const Eigen::Matrix<double, NX, 1>> lambda(in[2]);
            const Eigen::VectorXd z0 = stage(in[0], in[1]);
            vector_jacobian(
                [p](const auto& z) {
                    using T = typename std::decay_t<decltype(z)>::Scalar;
                    return Eigen::Matrix<T, Eigen::Dynamic, 1>(P::template dynamics<T>(state(z), input(z), p));
                },
                z0,
                out[0],
                out[1]);
            double lagrangian = 0.0;
            scalar_derivatives(
                [p, &lambda](const auto& z) {
                    using T = typename std::decay_t<decltype(z)>::Scalar;
                    return T(lambda.template cast<T>().dot(P::template dynamics<T>(state(z), input(z), p)));
                },
                z0,
                lagrangian,
                nullptr,
                out[2]);
        }

        // Stage cost: inputs (x, u, z, p); outputs the cost, its gradient, its Hessian, and the empty z blocks
        static double stage_cost(const Eigen::VectorXd& z, const double* p, double* gradient, double* hessian) {
            double value = 0.0;
            const auto f = [p](const auto& z) {
                using T = typename std::decay_t<decltype(z)>::Scalar;
                return T(P::template cost<T>(state(z), input(z), p));
            };
            if (hessian != nullptr) {
                scalar_derivatives(f, z, value, gradient, hessian);
            }
            else if (gradient != nullptr) {
                scalar_gradient(f, z, value, gradient);
            }
            else {
                value = P::template cost<double>(state(z), input(z), p);
            }
            return value;
        }
        static void cost(const double** in, double** out) {
            out[0][0] = stage_cost(stage(in[0], in[1]), in[3], nullptr, nullptr);
        }
        static void cost_jacobian(const double** in, double** out) {
            out[0][0] = stage_cost(stage(in[0], in[1]), in[3], out[1], nullptr);
        }
        static void cost_hessian(const double** in, double** out) {
            out[0][0] = stage_cost(stage(in[0], in[1]), in[3], out[1], out[2]);
        }

        // Terminal cost: inputs (x, u, z, p) with u and z empty, so z = x
        static double end_cost(const Eigen::VectorXd& x, const double* p, double* gradient, double* hessian) {
            double value = 0.0;
            const auto f = [p](const auto& x) {
                using T = typename std::decay_t<decltype(x)>::Scalar;
                return T(P::template terminal_cost<T>(State<T>(x), p));
            };
            if (hessian != nullptr) {
                scalar_derivatives(f, x, value, gradient, hessian);
            }
            else if (gradient != nullptr) {
                scalar_gradient(f, x, value, gradient);
            }
            else {
                value = P::template terminal_cost<double>(State<double>(x), p);
            }
            return value;
        }
        static void terminal_cost(const double** in, double** out) {
            out[0][0] = end_cost(Eigen::Map<const Eigen::VectorXd>(in[0], NX), in[3], nullptr, nullptr);
        }
        static void terminal_cost_jacobian(const double** in, double** out) {
            out[0][0] = end_cost(Eigen::Map<const Eigen::VectorXd>(in[0], NX), in[3], out[1], nullptr);
        }
        static void terminal_cost_hessian(const double** in, double** out) {
            out[0][0] = end_cost(Eigen::Map<const Eigen::VectorXd>(in[0], NX), in[3], out[1], out[2]);
        }

        // Constraints: inputs (x, u, z, p), or (x, u, λ, z, p) for the Hessian of λᵀh; outputs h, its transposed
        // Jacobian, its Hessian, and the empty z blocks
        template <bool TERMINAL>
        static auto constraint_of(const double* p) {
            return [p](const auto& z) {
                using T = typename std::decay_t<decltype(z)>::Scalar;
                if constexpr (TERMINAL) {
                    return Eigen::Matrix<T, Eigen::Dynamic, 1>(P::template terminal_constraint<T>(State<T>(z), p));
                }
                else {
                    return Eigen::Matrix<T, Eigen::Dynamic, 1>(P::template constraint<T>(state(z), input(z), p));
                }
            };
        }
        template <bool TERMINAL>
        static Eigen::VectorXd constraint_point(const double* x, const double* u) {
            return TERMINAL ? Eigen::VectorXd(Eigen::Map<const Eigen::VectorXd>(x, NX)) : stage(x, u);
        }
        template <bool TERMINAL>
        static void constraint(const double** in, double** out) {
            const Eigen::VectorXd h = constraint_of<TERMINAL>(in[3])(constraint_point<TERMINAL>(in[0], in[1]));
            Eigen::Map<Eigen::VectorXd>(out[0], NH) = h;
        }
        template <bool TERMINAL>
        static void constraint_jacobian(const double** in, double** out) {
            vector_jacobian(constraint_of<TERMINAL>(in[3]), constraint_point<TERMINAL>(in[0], in[1]), out[0], out[1]);
        }
        template <bool TERMINAL>
        static void constraint_hessian(const double** in, double** out) {
            const double* p = in[4];
            const Eigen::Map<const Eigen::Matrix<double, NH, 1>> lambda(in[2]);
            const Eigen::VectorXd z0 = constraint_point<TERMINAL>(in[0], in[1]);
            vector_jacobian(constraint_of<TERMINAL>(p), z0, out[0], out[1]);
            double lagrangian = 0.0;
            scalar_derivatives(
                [p, &lambda](const auto& z) {
                    using T = typename std::decay_t<decltype(z)>::Scalar;
                    return T(lambda.template cast<T>().dot(constraint_of<TERMINAL>(p)(z)));
                },
                z0,
                lagrangian,
                nullptr,
                out[2]);
        }

        /// @brief Every function, as acados's external functions
        static Functions functions() {
            constexpr Shape x      = dense(NX);
            constexpr Shape u      = dense(NU);
            constexpr Shape p      = dense(NP);
            constexpr Shape none   = dense(0);
            constexpr Shape scalar = dense(1);
            constexpr Shape h      = dense(NH);

            constexpr Signature<3, 1> dyn{{x, u, p}, {x}};
            constexpr Signature<3, 2> dyn_jac{{x, u, p}, {x, dense(NUX, NX)}};
            constexpr Signature<4, 3> dyn_hess{{x, u, x, p}, {x, dense(NUX, NX), dense(NUX, NUX)}};

            constexpr Signature<4, 1> cost_fun{{x, u, none, p}, {scalar}};
            constexpr Signature<4, 2> cost_jac{{x, u, none, p}, {scalar, dense(NUX)}};
            constexpr Signature<4, 5> cost_hess{{x, u, none, p},
                                                {scalar, dense(NUX), dense(NUX, NUX), dense(0, 0), dense(0, NUX)}};

            constexpr Signature<4, 1> end_fun{{x, none, none, p}, {scalar}};
            constexpr Signature<4, 2> end_jac{{x, none, none, p}, {scalar, dense(NX)}};
            constexpr Signature<4, 5> end_hess{{x, none, none, p},
                                               {scalar, dense(NX), dense(NX, NX), dense(0, 0), dense(0, NX)}};

            constexpr Signature<4, 1> h_fun{{x, u, none, p}, {h}};
            constexpr Signature<4, 3> h_jac{{x, u, none, p}, {h, dense(NUX, NH), dense(NH, 0)}};
            constexpr Signature<5, 5> h_hess{{x, u, h, none, p},
                                             {h, dense(NUX, NH), dense(NUX, NUX), dense(NH, 0), dense(0, 0)}};

            constexpr Signature<4, 1> h_end_fun{{x, none, none, p}, {h}};
            constexpr Signature<4, 3> h_end_jac{{x, none, none, p}, {h, dense(NX, NH), dense(NH, 0)}};
            constexpr Signature<5, 5> h_end_hess{{x, none, h, none, p},
                                                 {h, dense(NX, NH), dense(NX, NX), dense(NH, 0), dense(0, 0)}};

            return {
                {external_function<dyn, &dynamics>(),
                 external_function<dyn_jac, &dynamics_jacobian>(),
                 external_function<dyn_hess, &dynamics_hessian>()},
                {external_function<cost_fun, &cost>(),
                 external_function<cost_jac, &cost_jacobian>(),
                 external_function<cost_hess, &cost_hessian>()},
                {external_function<end_fun, &terminal_cost>(),
                 external_function<end_jac, &terminal_cost_jacobian>(),
                 external_function<end_hess, &terminal_cost_hessian>()},
                {external_function<h_fun, &constraint<false>>(),
                 external_function<h_jac, &constraint_jacobian<false>>(),
                 external_function<h_hess, &constraint_hessian<false>>()},
                {external_function<h_end_fun, &constraint<true>>(),
                 external_function<h_end_jac, &constraint_jacobian<true>>(),
                 external_function<h_end_hess, &constraint_hessian<true>>()},
            };
        }
    };

}  // namespace module::planning::inverted_walk_mpc::ocp

#endif  // MODULE_PLANNING_INVERTEDMPCWALKPATH_OCP_OCPFUNCTIONS_HPP
