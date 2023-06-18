/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   InEKF.cpp
 *  @author Ross Hartley
 *  @brief  Source file for Invariant EKF
 *  @date   September 25, 2018
 **/

#include "InEKF.hpp"

namespace utility::math::filter::inekf {

    void remove_row_and_column(Eigen::MatrixXd& M, int index) {
        unsigned int dim_X = M.cols();

        M.block(index, 0, dim_X - index - 1, dim_X) = M.bottomRows(dim_X - index - 1).eval();
        M.block(0, index, dim_X, dim_X - index - 1) = M.rightCols(dim_X - index - 1).eval();
        M.conservativeResize(dim_X - 1, dim_X - 1);
    }

    // ------------ InEKF -------------
    // Default constructor
    InEKF::InEKF() {}

    // Constructor with initial state and noise params
    InEKF::InEKF(RobotState s, NoiseParams p) : state(s), noise_params(p) {}

    // Return robot's current state
    RobotState InEKF::get_state() {
        return state;
    }

    // Sets the filter's noise parameters
    void InEKF::set_noise_params(NoiseParams p) {
        noise_params = p;
    }

    // Sets the filter's state
    void InEKF::set_state(RobotState s) {
        state = s;
    }

    // Set the filter's prior (static) landmarks
    void InEKF::set_prior_landmarks(const map_int_vec3d& pl) {
        prior_landmarks = pl;
    }

    // Return filter's estimated landmarks
    std::map<int, int> InEKF::get_estimated_landmarks() {
        return estimated_landmarks;
    }

    // Return filter's estimated landmarks
    std::map<int, int> InEKF::get_estimated_contact_positions() {
        return estimated_contact_positions;
    }

    // Set the filter's contact state
    void InEKF::set_contacts(std::vector<std::pair<int, bool>> c) {
        // Insert new measured contact states
        for (std::vector<std::pair<int, bool>>::iterator it = c.begin(); it != c.end(); ++it) {
            std::pair<std::map<int, bool>::iterator, bool> ret = contacts.insert(*it);
            // If contact is already in the map, replace with new value
            if (ret.second == false) {
                ret.first->second = it->second;
            }
        }
        return;
    }

    // InEKF Propagation - Inertial Data
    void InEKF::propagate(const Eigen::Vector3d& gyro, const Eigen::Vector3d& acc, double dt) {

        Eigen::Vector3d w = gyro - state.get_gyroscope_bias();     // Angular Velocity
        Eigen::Vector3d a = acc - state.get_accelerometer_bias();  // Linear Acceleration

        Eigen::MatrixXd X = state.get_X();
        Eigen::MatrixXd P = state.get_P();

        // Extract State
        Eigen::Matrix3d R = state.get_rotation();
        Eigen::Vector3d v = state.get_velocity();
        Eigen::Vector3d p = state.get_position();

        // Strapdown IMU motion model
        Eigen::Vector3d phi    = w * dt;
        Eigen::Matrix3d R_pred = R * exp_so3(phi);
        Eigen::Vector3d v_pred = v + (R * a + gravity) * dt;
        Eigen::Vector3d p_pred = p + v * dt + 0.5 * (R * a + gravity) * dt * dt;

        // Set new state (bias has constant dynamics)
        state.set_rotation(R_pred);
        state.set_velocity(v_pred);
        state.set_position(p_pred);

        // ---- Linearized invariant error dynamics -----
        int dim_X         = state.dim_X();
        int dim_P         = state.dim_P();
        int dim_theta     = state.dim_theta();
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(dim_P, dim_P);
        // Inertial terms
        A.block<3, 3>(3, 0) =
            skew(gravity);  // TODO: Efficiency could be improved by not computing the constant terms every time
        A.block<3, 3>(6, 3) = Eigen::Matrix3d::Identity();
        // Bias terms
        A.block<3, 3>(0, dim_P - dim_theta)     = -R;
        A.block<3, 3>(3, dim_P - dim_theta + 3) = -R;
        for (int i = 3; i < dim_X; ++i) {
            A.block<3, 3>(3 * i - 6, dim_P - dim_theta) = -skew(X.block<3, 1>(0, i)) * R;
        }

        // Noise terms
        Eigen::MatrixXd Qk   = Eigen::MatrixXd::Zero(dim_P, dim_P);  // Landmark noise terms will remain zero
        Qk.block<3, 3>(0, 0) = noise_params.get_gyroscope_cov();
        Qk.block<3, 3>(3, 3) = noise_params.get_accelerometer_cov();
        for (std::map<int, int>::iterator it = estimated_contact_positions.begin();
             it != estimated_contact_positions.end();
             ++it) {
            Qk.block<3, 3>(3 + 3 * (it->second - 3), 3 + 3 * (it->second - 3)) =
                noise_params.get_contact_cov();  // Contact noise terms
        }
        Qk.block<3, 3>(dim_P - dim_theta, dim_P - dim_theta)         = noise_params.get_gyroscope_bias_cov();
        Qk.block<3, 3>(dim_P - dim_theta + 3, dim_P - dim_theta + 3) = noise_params.get_accelerometer_bias_cov();

        // Discretization
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dim_P, dim_P);
        Eigen::MatrixXd Phi =
            I + A * dt;  // Fast approximation of exp(A*dt). TODO: explore using the full exp() instead
        Eigen::MatrixXd Adj                                   = I;
        Adj.block(0, 0, dim_P - dim_theta, dim_P - dim_theta) = adjoint_sek3(X);  // Approx 200 microseconds
        Eigen::MatrixXd PhiAdj                                = Phi * Adj;
        Eigen::MatrixXd Qk_hat                                = PhiAdj * Qk * PhiAdj.transpose()
                                 * dt;  // Approximated discretized noise matrix (faster by 400 microseconds)

        // Propagate Covariance
        Eigen::MatrixXd P_pred = Phi * P * Phi.transpose() + Qk_hat;

        // Set new covariance
        state.set_P(P_pred);

        return;
    }

    // Correct State: Right-Invariant Observation
    void InEKF::correct(const Observation& obs) {
        // Compute Kalman Gain
        Eigen::MatrixXd P   = state.get_P();
        Eigen::MatrixXd PHT = P * obs.H.transpose();
        Eigen::MatrixXd S   = obs.H * PHT + obs.N;
        Eigen::MatrixXd K   = PHT * S.inverse();

        // Copy X along the diagonals if more than one measurement
        Eigen::MatrixXd big_X;
        state.copy_diag_X(obs.Y.rows() / state.dim_X(), big_X);

        // Compute correction terms
        Eigen::MatrixXd Z      = big_X * obs.Y - obs.b;
        Eigen::VectorXd delta  = K * obs.PI * Z;
        Eigen::MatrixXd dX     = exp_sek3(delta.segment(0, delta.rows() - state.dim_theta()));
        Eigen::VectorXd dTheta = delta.segment(delta.rows() - state.dim_theta(), state.dim_theta());

        // Update state
        Eigen::MatrixXd X_new     = dX * state.get_X();  // Right-Invariant Update
        Eigen::VectorXd Theta_new = state.get_theta() + dTheta;
        state.set_X(X_new);
        state.set_theta(Theta_new);

        // Update Covariance
        Eigen::MatrixXd IKH   = Eigen::MatrixXd::Identity(state.dim_P(), state.dim_P()) - K * obs.H;
        Eigen::MatrixXd P_new = IKH * P * IKH.transpose() + K * obs.N * K.transpose();  // Joseph update form

        state.set_P(P_new);
    }

    // Create Observation from vector of landmark measurements
    void InEKF::correct_landmarks(const landmarks& measured_landmarks) {
        Eigen::VectorXd Y;
        Eigen::VectorXd b;
        Eigen::MatrixXd H;
        Eigen::MatrixXd N;
        Eigen::MatrixXd PI;

        Eigen::Matrix3d R = state.get_rotation();
        landmarks new_landmarks;
        std::vector<int> used_landmark_ids;

        for (landmarks_it it = measured_landmarks.begin(); it != measured_landmarks.end(); ++it) {
            // Detect and skip if an ID is not unique (this would cause singularity issues in InEKF::Correct)
            if (find(used_landmark_ids.begin(), used_landmark_ids.end(), it->id) != used_landmark_ids.end()) {
                std::cout << "Duplicate landmark ID detected! Skipping measurement.\n";
                continue;
            }
            else {
                used_landmark_ids.push_back(it->id);
            }

            // See if we can find id in prior_landmarks or estimated_landmarks
            map_int_vec3d_it it_prior                 = prior_landmarks.find(it->id);
            std::map<int, int>::iterator it_estimated = estimated_landmarks.find(it->id);
            if (it_prior != prior_landmarks.end()) {
                // Found in prior landmark set
                int dim_X = state.dim_X();
                int dim_P = state.dim_P();
                int start_index;

                // Fill out Y
                start_index = Y.rows();
                Y.conservativeResize(start_index + dim_X, Eigen::NoChange);
                Y.segment(start_index, dim_X) = Eigen::VectorXd::Zero(dim_X);
                Y.segment(start_index, 3)     = it->position;  // p_bl
                Y(start_index + 4)            = 1;

                // Fill out b
                start_index = b.rows();
                b.conservativeResize(start_index + dim_X, Eigen::NoChange);
                b.segment(start_index, dim_X) = Eigen::VectorXd::Zero(dim_X);
                b.segment(start_index, 3)     = it_prior->second;  // p_wl
                b(start_index + 4)            = 1;

                // Fill out H
                start_index = H.rows();
                H.conservativeResize(start_index + 3, dim_P);
                H.block(start_index, 0, 3, dim_P) = Eigen::MatrixXd::Zero(3, dim_P);
                H.block(start_index, 0, 3, 3)     = skew(it_prior->second);        // skew(p_wl)
                H.block(start_index, 6, 3, 3)     = -Eigen::Matrix3d::Identity();  // -I

                // Fill out N
                start_index = N.rows();
                N.conservativeResize(start_index + 3, start_index + 3);
                N.block(start_index, 0, 3, start_index) = Eigen::MatrixXd::Zero(3, start_index);
                N.block(0, start_index, start_index, 3) = Eigen::MatrixXd::Zero(start_index, 3);
                N.block(start_index, start_index, 3, 3) = R * noise_params.get_landmark_cov() * R.transpose();

                // Fill out PI
                start_index      = PI.rows();
                int start_index2 = PI.cols();
                PI.conservativeResize(start_index + 3, start_index2 + dim_X);
                PI.block(start_index, 0, 3, start_index2)     = Eigen::MatrixXd::Zero(3, start_index2);
                PI.block(0, start_index2, start_index, dim_X) = Eigen::MatrixXd::Zero(start_index, dim_X);
                PI.block(start_index, start_index2, 3, dim_X) = Eigen::MatrixXd::Zero(3, dim_X);
                PI.block(start_index, start_index2, 3, 3)     = Eigen::Matrix3d::Identity();
            }
            else if (it_estimated != estimated_landmarks.end()) {
                // Found in estimated landmark set
                int dim_X = state.dim_X();
                int dim_P = state.dim_P();
                int start_index;

                // Fill out Y
                start_index = Y.rows();
                Y.conservativeResize(start_index + dim_X, Eigen::NoChange);
                Y.segment(start_index, dim_X)         = Eigen::VectorXd::Zero(dim_X);
                Y.segment(start_index, 3)             = it->position;  // p_bl
                Y(start_index + 4)                    = 1;
                Y(start_index + it_estimated->second) = -1;

                // Fill out b
                start_index = b.rows();
                b.conservativeResize(start_index + dim_X, Eigen::NoChange);
                b.segment(start_index, dim_X)         = Eigen::VectorXd::Zero(dim_X);
                b(start_index + 4)                    = 1;
                b(start_index + it_estimated->second) = -1;

                // Fill out H
                start_index = H.rows();
                H.conservativeResize(start_index + 3, dim_P);
                H.block(start_index, 0, 3, dim_P)                        = Eigen::MatrixXd::Zero(3, dim_P);
                H.block(start_index, 6, 3, 3)                            = -Eigen::Matrix3d::Identity();  // -I
                H.block(start_index, 3 * it_estimated->second - 6, 3, 3) = Eigen::Matrix3d::Identity();   // I

                // Fill out N
                start_index = N.rows();
                N.conservativeResize(start_index + 3, start_index + 3);
                N.block(start_index, 0, 3, start_index) = Eigen::MatrixXd::Zero(3, start_index);
                N.block(0, start_index, start_index, 3) = Eigen::MatrixXd::Zero(start_index, 3);
                N.block(start_index, start_index, 3, 3) = R * noise_params.get_landmark_cov() * R.transpose();

                // Fill out PI
                start_index      = PI.rows();
                int start_index2 = PI.cols();
                PI.conservativeResize(start_index + 3, start_index2 + dim_X);
                PI.block(start_index, 0, 3, start_index2)     = Eigen::MatrixXd::Zero(3, start_index2);
                PI.block(0, start_index2, start_index, dim_X) = Eigen::MatrixXd::Zero(start_index, dim_X);
                PI.block(start_index, start_index2, 3, dim_X) = Eigen::MatrixXd::Zero(3, dim_X);
                PI.block(start_index, start_index2, 3, 3)     = Eigen::Matrix3d::Identity();
            }
            else {
                // First time landmark as been detected (add to list for later state augmentation)
                new_landmarks.push_back(*it);
            }
        }

        // Correct state using stacked observation
        Observation obs(Y, b, H, N, PI);
        if (!obs.empty()) {
            this->correct(obs);
        }

        // Augment state with newly detected landmarks
        if (new_landmarks.size() > 0) {
            Eigen::MatrixXd X_aug = state.get_X();
            Eigen::MatrixXd P_aug = state.get_P();
            Eigen::Vector3d p     = state.get_position();
            for (landmarks_it it = new_landmarks.begin(); it != new_landmarks.end(); ++it) {
                // Initialize new landmark mean
                int start_index = X_aug.rows();
                X_aug.conservativeResize(start_index + 1, start_index + 1);
                X_aug.block(start_index, 0, 1, start_index) = Eigen::MatrixXd::Zero(1, start_index);
                X_aug.block(0, start_index, start_index, 1) = Eigen::MatrixXd::Zero(start_index, 1);
                X_aug(start_index, start_index)             = 1;
                X_aug.block(0, start_index, 3, 1)           = p + R * it->position;

                // Initialize new landmark covariance - TODO:speed up
                Eigen::MatrixXd F = Eigen::MatrixXd::Zero(state.dim_P() + 3, state.dim_P());
                F.block(0, 0, state.dim_P() - state.dim_theta(), state.dim_P() - state.dim_theta()) =
                    Eigen::MatrixXd::Identity(state.dim_P() - state.dim_theta(),
                                              state.dim_P() - state.dim_theta());                   // for old X
                F.block(state.dim_P() - state.dim_theta(), 6, 3, 3) = Eigen::Matrix3d::Identity();  // for new landmark
                F.block(state.dim_P() - state.dim_theta() + 3,
                        state.dim_P() - state.dim_theta(),
                        state.dim_theta(),
                        state.dim_theta()) =
                    Eigen::MatrixXd::Identity(state.dim_theta(), state.dim_theta());  // for theta
                Eigen::MatrixXd G                                  = Eigen::MatrixXd::Zero(F.rows(), 3);
                G.block(G.rows() - state.dim_theta() - 3, 0, 3, 3) = R;
                P_aug = (F * P_aug * F.transpose() + G * noise_params.get_landmark_cov() * G.transpose()).eval();

                // Update state and covariance
                state.set_X(X_aug);
                state.set_P(P_aug);

                // Add to list of estimated landmarks
                estimated_landmarks.insert(std::pair<int, int>(it->id, start_index));
            }
        }
        return;
    }

    // Correct state using kinematics measured between imu and contact point
    void InEKF::correct_kinematics(const kinematics& measured_kinematics) {
        Eigen::VectorXd Y;
        Eigen::VectorXd b;
        Eigen::MatrixXd H;
        Eigen::MatrixXd N;
        Eigen::MatrixXd PI;

        Eigen::Matrix3d R = state.get_rotation();
        std::vector<std::pair<int, int>> remove_contacts;
        kinematics new_contacts;
        std::vector<int> used_contact_ids;

        for (kinematics_it it = measured_kinematics.begin(); it != measured_kinematics.end(); ++it) {
            // Detect and skip if an ID is not unique (this would cause singularity issues in InEKF::Correct)
            if (find(used_contact_ids.begin(), used_contact_ids.end(), it->id) != used_contact_ids.end()) {
                std::cout << "Duplicate contact ID detected! Skipping measurement.\n";
                continue;
            }
            else {
                used_contact_ids.push_back(it->id);
            }

            // Find contact indicator for the kinematics measurement
            std::map<int, bool>::iterator it_contact = contacts.find(it->id);
            if (it_contact == contacts.end()) {
                continue;
            }  // Skip if contact state is unknown
            bool contact_indicated = it_contact->second;

            // See if we can find id estimated_contact_positions
            std::map<int, int>::iterator it_estimated = estimated_contact_positions.find(it->id);
            bool found                                = it_estimated != estimated_contact_positions.end();

            // If contact is not indicated and id is found in estimated_contacts, then remove state
            if (!contact_indicated && found) {
                remove_contacts.push_back(*it_estimated);  // Add id to remove list
                //  If contact is indicated and id is not found i n estimated_contacts, then augment state
            }
            else if (contact_indicated && !found) {
                new_contacts.push_back(*it);  // Add to augment list

                // If contact is indicated and id is found in estimated_contacts, then correct using kinematics
            }
            else if (contact_indicated && found) {
                int dim_X = state.dim_X();
                int dim_P = state.dim_P();
                int start_index;

                // Fill out Y
                start_index = Y.rows();
                Y.conservativeResize(start_index + dim_X, Eigen::NoChange);
                Y.segment(start_index, dim_X)         = Eigen::VectorXd::Zero(dim_X);
                Y.segment(start_index, 3)             = it->pose.block<3, 1>(0, 3);  // p_bc
                Y(start_index + 4)                    = 1;
                Y(start_index + it_estimated->second) = -1;

                // Fill out b
                start_index = b.rows();
                b.conservativeResize(start_index + dim_X, Eigen::NoChange);
                b.segment(start_index, dim_X)         = Eigen::VectorXd::Zero(dim_X);
                b(start_index + 4)                    = 1;
                b(start_index + it_estimated->second) = -1;

                // Fill out H
                start_index = H.rows();
                H.conservativeResize(start_index + 3, dim_P);
                H.block(start_index, 0, 3, dim_P)                        = Eigen::MatrixXd::Zero(3, dim_P);
                H.block(start_index, 6, 3, 3)                            = -Eigen::Matrix3d::Identity();  // -I
                H.block(start_index, 3 * it_estimated->second - 6, 3, 3) = Eigen::Matrix3d::Identity();   // I

                // Fill out N
                start_index = N.rows();
                N.conservativeResize(start_index + 3, start_index + 3);
                N.block(start_index, 0, 3, start_index) = Eigen::MatrixXd::Zero(3, start_index);
                N.block(0, start_index, start_index, 3) = Eigen::MatrixXd::Zero(start_index, 3);
                N.block(start_index, start_index, 3, 3) = R * it->covariance.block<3, 3>(3, 3) * R.transpose();

                // Fill out PI
                start_index      = PI.rows();
                int start_index2 = PI.cols();
                PI.conservativeResize(start_index + 3, start_index2 + dim_X);
                PI.block(start_index, 0, 3, start_index2)     = Eigen::MatrixXd::Zero(3, start_index2);
                PI.block(0, start_index2, start_index, dim_X) = Eigen::MatrixXd::Zero(start_index, dim_X);
                PI.block(start_index, start_index2, 3, dim_X) = Eigen::MatrixXd::Zero(3, dim_X);
                PI.block(start_index, start_index2, 3, 3)     = Eigen::Matrix3d::Identity();

                //  If contact is not indicated and id is found in estimated_contacts, then skip
            }
            else {
                continue;
            }
        }

        // Correct state using stacked observation
        Observation obs(Y, b, H, N, PI);
        if (!obs.empty()) {
            this->correct(obs);
        }

        // Remove contacts from state
        if (remove_contacts.size() > 0) {
            Eigen::MatrixXd X_rem = state.get_X();
            Eigen::MatrixXd P_rem = state.get_P();
            for (std::vector<std::pair<int, int>>::iterator it = remove_contacts.begin(); it != remove_contacts.end();
                 ++it) {
                // Remove from list of estimated contact positions
                estimated_contact_positions.erase(it->first);

                // Remove row and column from X
                remove_row_and_column(X_rem, it->second);

                // Remove 3 rows and columns from P
                int start_index = 3 + 3 * (it->second - 3);
                remove_row_and_column(P_rem, start_index);  // TODO: Make more efficient
                remove_row_and_column(P_rem, start_index);  // TODO: Make more efficient
                remove_row_and_column(P_rem, start_index);  // TODO: Make more efficient

                // Update all indices for estimated_landmarks and estimated_contact_positions
                for (std::map<int, int>::iterator it2 = estimated_landmarks.begin(); it2 != estimated_landmarks.end();
                     ++it2) {
                    if (it2->second > it->second)
                        it2->second -= 1;
                }
                for (std::map<int, int>::iterator it2 = estimated_contact_positions.begin();
                     it2 != estimated_contact_positions.end();
                     ++it2) {
                    if (it2->second > it->second)
                        it2->second -= 1;
                }
                // We also need to update the indices of remove_contacts in the case where multiple contacts are being
                // removed at once
                for (std::vector<std::pair<int, int>>::iterator it2 = it; it2 != remove_contacts.end(); ++it2) {
                    if (it2->second > it->second)
                        it2->second -= 1;
                }

                // Update state and covariance
                state.set_X(X_rem);
                state.set_P(P_rem);
            }
        }


        // Augment state with newly detected contacts
        if (new_contacts.size() > 0) {
            Eigen::MatrixXd X_aug = state.get_X();
            Eigen::MatrixXd P_aug = state.get_P();
            Eigen::Vector3d p     = state.get_position();
            for (kinematics_it it = new_contacts.begin(); it != new_contacts.end(); ++it) {
                // Initialize new landmark mean
                int start_index = X_aug.rows();
                X_aug.conservativeResize(start_index + 1, start_index + 1);
                X_aug.block(start_index, 0, 1, start_index) = Eigen::MatrixXd::Zero(1, start_index);
                X_aug.block(0, start_index, start_index, 1) = Eigen::MatrixXd::Zero(start_index, 1);
                X_aug(start_index, start_index)             = 1;
                X_aug.block(0, start_index, 3, 1)           = p + R * it->pose.block<3, 1>(0, 3);

                // Initialize new landmark covariance - TODO:speed up
                Eigen::MatrixXd F = Eigen::MatrixXd::Zero(state.dim_P() + 3, state.dim_P());
                F.block(0, 0, state.dim_P() - state.dim_theta(), state.dim_P() - state.dim_theta()) =
                    Eigen::MatrixXd::Identity(state.dim_P() - state.dim_theta(),
                                              state.dim_P() - state.dim_theta());                   // for old X
                F.block(state.dim_P() - state.dim_theta(), 6, 3, 3) = Eigen::Matrix3d::Identity();  // for new landmark
                F.block(state.dim_P() - state.dim_theta() + 3,
                        state.dim_P() - state.dim_theta(),
                        state.dim_theta(),
                        state.dim_theta()) =
                    Eigen::MatrixXd::Identity(state.dim_theta(), state.dim_theta());  // for theta
                Eigen::MatrixXd G                                  = Eigen::MatrixXd::Zero(F.rows(), 3);
                G.block(G.rows() - state.dim_theta() - 3, 0, 3, 3) = R;
                P_aug = (F * P_aug * F.transpose() + G * it->covariance.block<3, 3>(3, 3) * G.transpose()).eval();

                // Update state and covariance
                state.set_X(X_aug);
                state.set_P(P_aug);

                // Add to list of estimated contact positions
                estimated_contact_positions.insert(std::pair<int, int>(it->id, start_index));
            }
        }

        return;
    }


}  // namespace utility::math::filter::inekf
