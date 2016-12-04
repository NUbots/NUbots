/*
author Jake Fountain
This code is part of mocap-kinect experiments.
The sensor plant is responsible for fusing multiple measurements*/
#include <armadillo>
#include <chrono>
#include <string>
#include <set>
#include <queue>
#include <map>
#include "MocapStream.h"
#include "MocapRecording.h"
#include "CalibrationTools.h"
#include "Correlator.h"
#include "Simulation.h"

#ifndef AUTOCAL_SENSOR_PLANT
#define AUTOCAL_SENSOR_PLANT

namespace autocal {
	
	class SensorPlant{

		using Hypothesis = std::pair<int,int>;
		using NamePair = std::pair<std::string,std::string>;
				
		std::map<NamePair, utility::math::matrix::Transform3D> groundTruthTransforms;

		std::map<NamePair ,Correlator> correlators;

		//State variables
		std::queue<SimulationParameters> simParams;
		arma::running_stat<double> computeTimes;
		std::map<int, int> correctGuesses; 
		std::map<int, int> totalGuesses; 
		std::map<NamePair,std::map<int, int>> correctMatchings;



	public:
		SensorPlant(){}

		MocapRecording mocapRecording;

		const MocapStream& getStream(std::string name){
			return mocapRecording.getStream(name);
		}

		void addStream(const MocapStream& s);

		bool streamNotEmpty(std::string name){
			return !getStream(name).isEmpty();
		}
								
		std::vector<Hypothesis> matchStreams(std::string stream_name_1, std::string stream_name_2, TimeStamp now, TimeStamp latencyOfStream1 = 0);

		std::map<MocapStream::RigidBodyID,float> multiply(std::map<MocapStream::RigidBodyID,float> m1, std::map<MocapStream::RigidBodyID,float> m2);

		void setGroundTruthTransform(std::string streamA, std::string streamB, utility::math::matrix::Transform3D mapAtoB, bool useTruth = false);
		
		void setAnswers(std::string s1, std::string s2, std::map<int,int> answers);
		
		void setSimParameters(
			SimulationParameters a1, SimulationParameters a2, int aN,
			SimulationParameters d1, SimulationParameters d2, int dN);

		bool next();

		void convertToGroundTruth(std::string streamA, std::string streamB);

		autocal::MocapStream::Frame getGroundTruth(std::string stream, std::string desiredBasis, TimeStamp now);

		void setCurrentSimParameters(const SimulationParameters& sim);


	};

}
#endif