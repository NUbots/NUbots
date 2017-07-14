/*
author Jake Fountain
This code is part of mocap-kinect experiments*/
#include <dirent.h>
#include <armadillo>
#include <chrono>
#include <map>
#include <set>
#include "CalibrationTools.h"
#include "MocapStream.h"
#include "utility/math/geometry/UnitQuaternion.h"
#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/matrix/Transform3D.h"

#ifndef AUTOCAL_CORRELATOR
#define AUTOCAL_CORRELATOR

namespace autocal {

class Correlator {
public:
    Correlator();
    ~Correlator(){};

private:
    // CONFIG
    int number_of_samples;

    float difference_threshold;

    float elimination_score_threshold;

    float score_inclusion_threshold;
    // STATE

    using Hypothesis = std::pair<MocapStream::RigidBodyID, MocapStream::RigidBodyID>;
    using Stream     = std::vector<utility::math::matrix::Transform3D>;
    using StreamPair = std::pair<Stream, Stream>;

    // States for matchStreams
    std::map<Hypothesis, StreamPair> recordedStates;
    // Stores scores for the matchings
    std::map<Hypothesis, float> scores;
    // Stores the matches which have been deduced incorrect
    std::set<Hypothesis> eliminatedHypotheses;

    std::set<Hypothesis> computableStreams;

    // For rotation scoring:
    std::map<Hypothesis, std::pair<utility::math::matrix::Rotation3D, utility::math::matrix::Rotation3D>>
        firstRotationReadings;

    float getSylvesterScore(const Stream& states1, const Stream& states2, Hypothesis key);

    float getRotationScore(const Stream& states1, const Stream& states2, Hypothesis key);

    void resetRecordedStates();

    bool stateIsNew(const utility::math::matrix::Transform3D& T, const Stream& states);

public:
    void addData(MocapStream::RigidBodyID id1,
                 utility::math::matrix::Transform3D T1,
                 MocapStream::RigidBodyID id2,
                 utility::math::matrix::Transform3D T2);

    void eliminateAndNormalise(std::map<MocapStream::RigidBodyID, float> totalScores);

    bool sufficientData();

    void compute();

    void reset();

    std::vector<std::pair<int, int>> getBestCorrelations();

    float likelihood(float error) {
        return std::exp(-error * error);
    }
};
}  // namespace autocal
#endif