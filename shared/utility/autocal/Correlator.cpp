/*
author Jake Fountain
This code is part of mocap-kinect experiments*/

#include "Correlator.h"
#include <limits>
namespace autocal {

using utility::math::geometry::UnitQuaternion;
using utility::math::matrix::Rotation3D;
using utility::math::matrix::Transform3D;

Correlator::Correlator() : firstRotationReadings() {
    number_of_samples           = 10;
    difference_threshold        = 0.1;
    elimination_score_threshold = 0.1;
    score_inclusion_threshold   = 0.5;
}


void Correlator::addData(MocapStream::RigidBodyID id1, Transform3D T1, MocapStream::RigidBodyID id2, Transform3D T2) {
    // Generate key for the map of correlationStats
    std::pair<MocapStream::RigidBodyID, MocapStream::RigidBodyID> key = {id1, id2};

    // Check if this hypothesis has been eliminated
    if (eliminatedHypotheses.count(key) != 0) return;

    if (recordedStates.count(key) == 0) {
        // Initialise if key missing. (true => calculate cov)
        recordedStates[key] = StreamPair();
    }
    else {

        // std::cout << "number of samples = " << recordedStates[key].first.size() << std::endl;
        // std::cout << "diff1 = " << diff1 << std::endl;
        // if( id1 == 1 && id2 == 18) std::cout << "T2 = " << T2 << std::endl;
        // std::cout << "diff2 = " << diff2 << std::endl;
        // std::cout << "T2 = " << T2 << std::endl;

        // If we have no recorded states yet, or the new states differ significantly from the previous, add the new
        // states
        if (stateIsNew(T1, recordedStates[key].first) || stateIsNew(T2, recordedStates[key].second)) {
            // Check if bad sample (for the particular solver we are using):
            Rotation3D R1 = T1.rotation();
            UnitQuaternion q1(R1);
            Rotation3D R2 = T2.rotation();
            UnitQuaternion q2(R2);
            if (q1.kW() == 0 || q2.kW() == 0) return;

            // Check det
            if (std::fabs(arma::det(R1) - 1) > 0.1) {
                std::cout << __FILE__ << " : Det R1 = " << arma::det(R1) << std::endl;
            }
            if (std::fabs(arma::det(R2) - 1) > 0.1) {
                std::cout << __FILE__ << " : Det R2 = " << arma::det(R2) << std::endl;
            }

            std::cout << "Recording Sample..." << std::endl;

            if (recordedStates[key].first.size() >= number_of_samples) {
                recordedStates[key].first.erase(recordedStates[key].first.begin());
                recordedStates[key].first.push_back(T1);
                recordedStates[key].second.erase(recordedStates[key].second.begin());
                recordedStates[key].second.push_back(T2);
                // Now we are ready to compute
                computableStreams.insert(key);
            }
            else {
                recordedStates[key].first.push_back(T1);
                recordedStates[key].second.push_back(T2);
            }
        }
    }
}

void Correlator::eliminateAndNormalise(std::map<MocapStream::RigidBodyID, float> totalScores) {
    // Normalise scores and eliminate low scores
    for (auto& s : scores) {
        const auto& pairID = s.first;
        const auto& id1    = pairID.first;
        float& score       = s.second;
        if (eliminatedHypotheses.count(pairID) != 0) continue;

        if (totalScores[id1] != 0) {
            // Normalise
            // score = score / totalScores[id1];
            // Eliminate
            if (score < elimination_score_threshold && eliminatedHypotheses.count(pairID) == 0) {
                eliminatedHypotheses.insert(pairID);
                std::cout << "Eliminated: [" << pairID.first << "," << pairID.second << "]" << std::endl;
            }
        }
    }
    // If all eliminated
    if (eliminatedHypotheses.size() == scores.size()) {
        reset();
    }
}

bool Correlator::sufficientData() {
    if (computableStreams.size() == 0) return false;
    for (auto& hypothesis : recordedStates) {
        const auto& key = hypothesis.first;
        // std::cout << "computableStreams.count(key) = " << computableStreams.count(key) << std::endl;
        if (eliminatedHypotheses.count(key) != 0) continue;
        if (computableStreams.count(key) == 0) return false;
    }
    return true;
}

void Correlator::compute() {
    std::cout << "\nComputing matches..." << std::endl;
    std::map<MocapStream::RigidBodyID, float> totalScores;
    for (auto& hypothesis : recordedStates) {
        // Extract data from its confusing encrypted data structure
        const auto& key                   = hypothesis.first;
        const auto& id1                   = key.first;
        const auto& id2                   = key.second;
        const Correlator::Stream& states1 = hypothesis.second.first;
        const Correlator::Stream& states2 = hypothesis.second.second;

        // Check whether or not we need to check this hypothesis anymore
        if (eliminatedHypotheses.count(key) != 0) continue;

        // Init total scores if necessary
        if (totalScores.count(id1) == 0) {
            totalScores[id1] = 0;
        }
        // CONFIG HERE:
        // CE METHOD
        float score = getSylvesterScore(states1, states2, key);
        // IF METHOD
        // float score = getRotationScore(states1, states2, key);


        // Init score to 1 if not recorded or set at zero
        if (scores.count(key) == 0 || scores[key] == 0) {
            scores[key] = 1;
        }

        // weight decay
        // CONFIG HERE: score accumulation turned on by uncommenting this line:
        scores[key] = score;  // * scores[key];

        std::cout << "score[" << id1 << "," << id2 << "] = " << scores[key] << " " << states1.size() << " samples "
                  << std::endl;
        // for(int i = 0; i < states1.size(); i++){
        // 	std::cout << "states1[" << i << "] \n" << states1[i] << std::endl;
        // 	std::cout << "states2[" << i << "] \n" << states2[i] << std::endl;
        // }

        totalScores[id1] += scores[key];
    }
    // std::cout << std::endl;
    computableStreams.clear();

    eliminateAndNormalise(totalScores);

    // resetRecordedStates();
}

void Correlator::reset() {
    resetRecordedStates();
    eliminatedHypotheses.clear();
    scores.clear();
    firstRotationReadings.clear();
}

void Correlator::resetRecordedStates() {
    recordedStates.clear();
    computableStreams.clear();
}

bool Correlator::stateIsNew(const utility::math::matrix::Transform3D& T, const Stream& states) {
    if (states.empty()) return true;

    // OLD METHOD
    // Add current stats to the vector
    const Transform3D& lastTransform = states.back().i();
    float diff                       = Transform3D::norm(lastTransform * T);
    return diff > difference_threshold;

    // New method
    // float minDiffAngle = std::numeric_limits<float>::max();
    // float minDiffPos = std::numeric_limits<float>::max();
    // for(auto& S : states){
    // 	float diffAngle = Rotation3D::norm(S.rotation().t() * T.rotation());
    // 	float diffPos = arma::norm(T.translation() - S.translation());
    // 	if(diffAngle < minDiffAngle && diffPos < minDiffPos ){
    // 		minDiffAngle = diffAngle;
    // 		minDiffPos = diffPos;
    // 	}
    // }
    // // std::cout << "angle = " << minDiffAngle << " pos = " << minDiffPos << std::endl;
    // return minDiffAngle > difference_threshold && minDiffPos > difference_threshold;
}


float Correlator::getSylvesterScore(const Correlator::Stream& states1,
                                    const Correlator::Stream& states2,
                                    Correlator::Hypothesis key) {
    // Fit data
    bool success = true;
    // auto result = CalibrationTools::solveZhuang1994(states1,states2,success);
    auto result = CalibrationTools::solveKronecker_Shah2013(states1, states2, success);

    // if the fit failed, return zero score
    if (!success) return 0;

    /////////////////////
    /////////////////////
    /////////////////////
    /////////////////////
    // TODO/NOTE
    // note that the equation for the system is
    // M Ai = Bi N
    // and so by putting the samples in order A,B we are actually solving
    // Ai N.i() = M.i() Bi
    // so
    // X = N.i()
    // and
    // Y = M.i()
    /////////////////////
    /////////////////////
    /////////////////////
    /////////////////////
    auto X = result.first;
    auto Y = result.second;

    // Calculate reprojection error as score
    float totalError = 0;

    for (int i = 0; i < states1.size(); i++) {
        const Transform3D& A = states1[i];
        const Transform3D& B = states2[i];
        Transform3D errorMat = (A * X).i() * (Y * B);
        float error          = Transform3D::norm(errorMat);
        totalError += error;
        UnitQuaternion rotA(Rotation3D(A.rotation()));
        UnitQuaternion rotB(Rotation3D(B.rotation()));
        // std::cout << "Error = " << error << std::endl
        // 		  << " A = " << rotA.getAngle() << " rads about axis " << rotA.getAxis().t()
        // 		  << " B = " << rotB.getAngle() << " rads about axis " << rotB.getAxis().t()
        // 		  << std::endl;
        // std::cout << "det(A.rotation()) = " << arma::det(A.rotation()) << std::endl;
        // std::cout << "det(B.rotation()) = " << arma::det(B.rotation()) << std::endl;
        // std::cout << "det(X.rotation()) = " << arma::det(X.rotation()) << std::endl;
        // std::cout << "det(Y.rotation()) = " << arma::det(Y.rotation()) << std::endl;
    }
    std::cout << "error = " << totalError / float(number_of_samples) << " per sample" << std::endl;
    return likelihood(totalError / float(number_of_samples));
}

float Correlator::getRotationScore(const Correlator::Stream& states1,
                                   const Correlator::Stream& states2,
                                   Correlator::Hypothesis key) {

    if (firstRotationReadings.count(key) == 0) {
        std::pair<utility::math::matrix::Rotation3D, utility::math::matrix::Rotation3D> val = {
            states1.front().rotation(), states2.front().rotation()};
        firstRotationReadings[key] = val;
        return 1;
    }

    // Fit data
    Rotation3D R1 = states1.back().rotation().t() * states1.front().rotation();
    Rotation3D R2 = states2.back().rotation().t() * states2.front().rotation();

    float angle1 = Rotation3D::norm(R1);
    float angle2 = Rotation3D::norm(R2);

    // std::cout << "angle for rb" << key.first <<" = " << angle1 << " angle for psmove "<< key.second << " = " <<
    // angle2 << std::endl;
    float error = std::fabs(angle2 - angle1);
    // BELOW LINE DOESNT MAKE A LARGE DIFFERENCE.
    error = std::fmin(error, 2 * M_PI - error);

    return likelihood(error);
}

std::vector<std::pair<int, int>> Correlator::getBestCorrelations() {
    // TODO: this
    std::map<MocapStream::RigidBodyID, MocapStream::RigidBodyID> bestMatches;
    std::map<MocapStream::RigidBodyID, float> bestScores;

    for (const auto& s : scores) {
        const auto& key    = s.first;
        const float& score = s.second;
        if (eliminatedHypotheses.count(key) != 0) continue;

        const auto& id1 = key.first;
        const auto& id2 = key.second;

        if (bestScores.count(id1) == 0) {
            bestScores[id1]  = score;
            bestMatches[id1] = id2;
        }

        if (score > bestScores[id1]) {
            bestScores[id1]  = score;
            bestMatches[id1] = id2;
        }
    }
    std::vector<std::pair<int, int>> result;
    for (auto& match : bestMatches) {
        if (bestScores[match.first] > score_inclusion_threshold) {
            result.push_back(std::make_pair(int(match.first), int(match.second)));
        }
    }
    return result;
}
}  // namespace autocal
