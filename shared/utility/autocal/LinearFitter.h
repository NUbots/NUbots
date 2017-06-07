/*
author Jake Fountain
This code is part of mocap-kinect experiments*/

#ifndef AUTOCAL_VELOCITY_FITTER
#define AUTOCAL_VELOCITY_FITTER

namespace autocal {
	template <int N>
	class LinearFitter {
	private:
        using Vec = arma::vec::fixed<N>;

		arma::mat data;
		arma::mat times;

		int number_of_samples;

		bool enough_samples;

	public:
		LinearFitter(int n){
			number_of_samples = n;
		}

		void addData(Vec new_data, double t_sec){
			data.insert_cols(0,new_data);
			times.insert_cols(0,arma::vec2({t_sec,1}));
			enough_samples = data.n_cols > number_of_samples;
			if(enough_samples){
				data.shed_col(data.n_cols-1);
				times.shed_col(times.n_cols-1);
			}
		}

		bool canFit() {
			return enough_samples;
		}

		arma::mat fit(){
			return data * arma::pinv(times);
		}
	};


}
#endif
