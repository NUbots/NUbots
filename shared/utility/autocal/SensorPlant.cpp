/*
author Jake Fountain
This code is part of mocap-kinect experiments*/

#include "SensorPlant.h"
#include <math.h>
#include <set>
#include "utility/math/matrix/Transform3D.h"

namespace autocal {
using utility::math::matrix::Transform3D;

// match each rigid body in stream 1 with a rigid body in stream 2
std::vector<SensorPlant::Hypothesis> SensorPlant::matchStreams(std::string stream_name_1,
                                                               std::string stream_name_2,
                                                               TimeStamp now,
                                                               TimeStamp latencyOfStream1) {
    // std::cout << "FRAME BEGIN"  << std::endl;
    auto start = std::chrono::high_resolution_clock::now();

    std::vector<SensorPlant::Hypothesis> empty_result;

    MocapStream& stream1 = mocapRecording.getStream(stream_name_1);
    MocapStream& stream2 = mocapRecording.getStream(stream_name_2);

    NamePair hypothesisKey({stream_name_1, stream_name_2});

    // Initialise eliminated hypotheses if necessary
    if (correlators.count(hypothesisKey) == 0) {
        correlators[hypothesisKey] = Correlator();
    }
    auto& correlator = correlators[hypothesisKey];

    // Check we have data to compare
    if (stream2.size() == 0 || stream1.size() == 0) {
        return empty_result;
    }

    std::map<MocapStream::RigidBodyID, Transform3D> currentState1 = stream1.getCompleteStates(now + latencyOfStream1);
    std::map<MocapStream::RigidBodyID, Transform3D> currentState2 = stream2.getCompleteStates(now);


    // Update statistics
    for (auto& state1 : currentState1) {
        // For each rigid body to be matched
        MocapStream::RigidBodyID id1 = state1.first;
        for (auto& state2 : currentState2) {
            // For each rigid body to match to
            MocapStream::RigidBodyID id2 = state2.first;

            correlator.addData(id1, state1.second, id2, state2.second);
        }
    }

    // Compute correlations
    if (correlator.sufficientData()) {
        correlator.compute();
    }

    std::vector<SensorPlant::Hypothesis> correlations = correlator.getBestCorrelations();

    auto finish = std::chrono::high_resolution_clock::now();
    computeTimes(double(std::chrono::duration_cast<std::chrono::nanoseconds>(finish - start).count() * 1e-6));

    // Compute correct guesses:
    auto answers = correctMatchings[hypothesisKey];
    for (auto& cor : correlations) {
        if (correctGuesses.count(cor.first) == 0) correctGuesses[cor.first] = 0;
        if (totalGuesses.count(cor.first) == 0) totalGuesses[cor.first] = 0;
        if (answers.count(cor.first) > 0) {
            correctGuesses[cor.first] += int(answers[cor.first] == cor.second);
        }
        totalGuesses[cor.first]++;
    }

    return correlations;
}

std::map<MocapStream::RigidBodyID, float> SensorPlant::multiply(std::map<MocapStream::RigidBodyID, float> m1,
                                                                std::map<MocapStream::RigidBodyID, float> m2) {
    std::map<MocapStream::RigidBodyID, float> result;
    float learningRate = 0.1;
    for (auto& x : m1) {
        if (m2.count(x.first) != 0) {
            // Exponential filter
            // result[x.first] = (1-learningRate) * m1[x.first] + learningRate * m2[x.first];

            // Probabilistic decay filter
            result[x.first] = m1[x.first] * std::pow(m2[x.first], 1 - learningRate);
        }
        else {
            result[x.first] = m1[x.first];
        }
    }
    // Add in keys present in m2 but not m1
    for (auto& x : m2) {
        if (m1.count(x.first) == 0) {
            result[x.first] = m2[x.first];
        }
    }
    return result;
}

void SensorPlant::setGroundTruthTransform(std::string streamA,
                                          std::string streamB,
                                          Transform3D mapAtoB,
                                          bool useTruth) {
    groundTruthTransforms[std::make_pair(streamA, streamB)] = mapAtoB;
    // HACK CORRECTION (for kinect, no longer necessary)
    // groundTruthTransforms[std::make_pair(streamA, streamB)].translation() += arma::vec3{-0.38,0,0};
    // std::cout << "groundTruthTransforms \n" << groundTruthTransforms[std::make_pair(streamA, streamB)]<<  std::endl;

    if (useTruth) {
        convertToGroundTruth(streamA, streamB);
        // Set to identity
        groundTruthTransforms[std::make_pair(streamA, streamB)] = Transform3D();
    }
}

void SensorPlant::convertToGroundTruth(std::string streamA, std::string streamB) {
    auto key = std::make_pair(streamA, streamB);

    if (groundTruthTransforms.count(key) != 0 && mocapRecording.streamPresent(streamA)) {
        // Get the transform between coordinate systems
        Transform3D streamToDesiredBasis = groundTruthTransforms[key];

        mocapRecording.getStream(streamA).transform(streamToDesiredBasis);
    }
    else {
        std::cout << "WARNING: ATTEMPTING TO ACCESSING GROUND TRUTH WHEN NONE EXISTS!!!" << std::endl;
    }
}

autocal::MocapStream::Frame SensorPlant::getGroundTruth(std::string stream, std::string desiredBasis, TimeStamp now) {
    // If we are transforming to the same reference basis, just return the current frame unaltered
    if (stream.compare(desiredBasis) == 0) {
        return mocapRecording.getStream(stream).getFrame(now);
    }
    // Init result otherwise
    autocal::MocapStream::Frame truth;
    // make the key to retrieve the ground truth transform
    // TODO: check alternate key ordering too (as its just a matrix inverse to swap order)
    auto key = std::make_pair(stream, desiredBasis);
    if (groundTruthTransforms.count(key) != 0 && mocapRecording.streamPresent(stream)) {
        // Get the transform between coordinate systems
        Transform3D streamToDesiredBasis = groundTruthTransforms[key];

        // Get the latest data
        MocapStream::Frame latestFrame = mocapRecording.getStream(stream).getFrame(now);

        // Loop through and record transformed rigid body poses
        for (auto& rb : latestFrame.rigidBodies) {
            truth.rigidBodies[rb.first].pose = streamToDesiredBasis * rb.second.pose;
        }
    }
    else {
        std::cout << "WARNING: ATTEMPTING TO ACCESS GROUND TRUTH WHEN NONE EXISTS!!!" << std::endl;
    }

    return truth;
}

void SensorPlant::addStream(const MocapStream& s) {
    mocapRecording.getStream(s.name()) = s;
    if (simParams.size() != 0) {
        mocapRecording.getStream(s.name()).setSimulationParameters(simParams.front());
    }
}

bool SensorPlant::next() {
    for (auto& c : correlators) {
        c.second.reset();
    }
    if (simParams.size() != 0) {
        SimulationParameters s = simParams.front();
        simParams.pop();
        std::cerr << "Finished simulating: " << s.latency_ms << " " << s.noise.angle_stddev << " "
                  << s.noise.disp_stddev << " " << s.slip.disp.f << " " << s.slip.disp.A << " " << s.slip.angle.f << " "
                  << s.slip.angle.A << " ";
    }
    std::cerr << " Fraction correct: " << std::endl;
    for (auto guess : correctGuesses) {
        std::cerr << "id: " << guess.first << " = " << float(guess.second) / float(totalGuesses[guess.first])
                  << std::endl;
    }
    std::cerr << " time= " << computeTimes.min() << " " << computeTimes.mean() << " " << computeTimes.max()
              << std::endl;
    correctGuesses.clear();
    totalGuesses.clear();
    computeTimes.reset();
    if (simParams.size() != 0) {
        setCurrentSimParameters(simParams.front());
    }
    return simParams.size() != 0;
}

void SensorPlant::setSimParameters(SimulationParameters a1,
                                   SimulationParameters a2,
                                   int aN,
                                   SimulationParameters d1,
                                   SimulationParameters d2,
                                   int dN) {

    simParams = std::queue<SimulationParameters>();  // clear queue

    SimulationParameters aStep;
    if (aN != 1) {
        aStep = (a2 - a1) * (1 / float(aN - 1));
    }

    SimulationParameters dStep;
    if (dN != 1) {
        dStep = (d2 - d1) * (1 / float(dN - 1));
    }

    for (int i = 0; i < aN; i++) {
        SimulationParameters a;
        a = a1 + aStep * i;
        for (int j = 0; j < dN; j++) {
            SimulationParameters d;
            d = d1 + dStep * j;

            simParams.push(a + d);
        }
    }
}

void SensorPlant::setAnswers(std::string s1, std::string s2, std::map<int, int> answers) {
    NamePair key          = NamePair({s1, s2});
    correctMatchings[key] = answers;
}

void SensorPlant::setCurrentSimParameters(const SimulationParameters& sim) {
    for (auto& stream : mocapRecording.streams) {
        stream.second.setSimulationParameters(sim);
    }
}
}  // namespace autocal
