/*
author Jake Fountain
This code is part of mocap-kinect experiments.
The sensor plant is responsible for fusing multiple measurements*/

#ifndef AUTOCAL_SIMULATION_H
#define AUTOCAL_SIMULATION_H
namespace autocal{

	struct SimulationParameters{
		
		struct SinFunc{
			float f = 0;//frequency
			float A = 0;//amplitude
		};

		struct Noise{
			float angle_stddev = 0;
			float disp_stddev = 0;
		};

		struct {
			SinFunc disp;
			SinFunc angle;
		} slip;
		float latency_ms = 0;
		Noise noise;

		SimulationParameters operator+(const SimulationParameters& s){
			SimulationParameters s_;
			
			s_.latency_ms = s.latency_ms + this->latency_ms;
			
			s_.noise.angle_stddev = s.noise.angle_stddev + this->noise.angle_stddev;
			s_.noise.disp_stddev = s.noise.disp_stddev + this->noise.disp_stddev;

			s_.slip.disp.f = s.slip.disp.f + this->slip.disp.f;
			s_.slip.disp.A = s.slip.disp.A + this->slip.disp.A;
			s_.slip.angle.f = s.slip.angle.f + this->slip.angle.f;
			s_.slip.angle.A = s.slip.angle.A + this->slip.angle.A;

			return s_;
		}
		SimulationParameters operator-(const SimulationParameters& s){
			SimulationParameters s_;
			
			s_.latency_ms = this->latency_ms - s.latency_ms ;
			
			s_.noise.angle_stddev = this->noise.angle_stddev - s.noise.angle_stddev ;
			s_.noise.disp_stddev = this->noise.disp_stddev - s.noise.disp_stddev ;

			s_.slip.disp.f = this->slip.disp.f - s.slip.disp.f ;
			s_.slip.disp.A = this->slip.disp.A - s.slip.disp.A ;
			s_.slip.angle.f = this->slip.angle.f - s.slip.angle.f ;
			s_.slip.angle.A = this->slip.angle.A - s.slip.angle.A ;

			return s_;
		}
		SimulationParameters operator*(const float& f){
			SimulationParameters s_;
			
			s_.latency_ms = this->latency_ms * f;
			
			s_.noise.angle_stddev = this->noise.angle_stddev * f;
			s_.noise.disp_stddev = this->noise.disp_stddev * f;

			s_.slip.disp.f = this->slip.disp.f * f;
			s_.slip.disp.A = this->slip.disp.A * f;
			s_.slip.angle.f = this->slip.angle.f * f;
			s_.slip.angle.A = this->slip.angle.A * f;

			return s_;
		}

	};

}

#endif