// Copyright 2008-2016 Conrad Sanderson (http://conradsanderson.id.au)
// Copyright 2008-2016 National ICT Australia (NICTA)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// ------------------------------------------------------------------------

#include <complex>
#include <limits>

namespace module {
namespace support {
    namespace optimisation {

        template <typename Scalar>
        class stat_counter {
        public:
            stat_counter() : d_count(Scalar(0)), i_count(size_t(0)) {}

            const stat_counter& operator++() {
                if (i_count < std::numeric_limits<size_t>::max()) {
                    i_count++;
                }
                else {
                    d_count += Scalar(std::numeric_limits<size_t>::max());
                    i_count = 1;
                }

                return *this;
            }
            void operator++(int) {
                operator++();
            }

            void reset() {
                d_count = Scalar(0);
                i_count = size_t(0);
            }
            Scalar value() const {
                return d_count + Scalar(i_count);
            }
            Scalar value_plus_1() const {
                if (i_count < std::numeric_limits<size_t>::max()) {
                    return d_count + Scalar(i_count + 1);
                }
                else {
                    return d_count + Scalar(std::numeric_limits<size_t>::max()) + Scalar(1);
                }
            }
            Scalar value_minus_1() const {
                if (i_count > 0) {
                    return d_count + Scalar(i_count - 1);
                }
                else {
                    return d_count - Scalar(1);
                }
            }

        private:
            Scalar d_count;
            size_t i_count;
        };

        //! Class for keeping statistics of a continuously sampled process / signal.
        //! Useful if the storage of individual samples is not necessary or desired.
        //! Also useful if the number of samples is not known beforehand or exceeds
        //! available memory.
        template <typename Scalar>
        class running_stat {
        public:
            running_stat()
                : r_mean(Scalar(0))
                , r_var(Scalar(0))
                , min_val(Scalar(0))
                , max_val(Scalar(0))
                , min_val_norm(Scalar(0))
                , max_val_norm(Scalar(0)) {}

            //! update statistics to reflect new sample
            void operator()(const Scalar& sample) {
                if (std::isfinite(sample)) {
                    update_stats(sample);
                }
            }

            //! set all statistics to zero
            void reset() {
                counter.reset();

                r_mean = Scalar(0);
                r_var  = Scalar(0);

                min_val = Scalar(0);
                max_val = Scalar(0);

                min_val_norm = Scalar(0);
                max_val_norm = Scalar(0);
            }

            //! mean or average value
            Scalar mean() const {
                return r_mean;
            }

            //! variance
            Scalar var(const size_t& norm_type = 0) const {
                const Scalar N = counter.value();

                if (N > Scalar(1)) {
                    if (norm_type == 0) {
                        return r_var;
                    }
                    else {
                        const Scalar N_minus_1 = counter.value_minus_1();
                        return (N_minus_1 / N) * r_var;
                    }
                }
                else {
                    return Scalar(0);
                }
            }
            //! standard deviation
            Scalar stddev(const size_t& norm_type = 0) const {
                return std::sqrt(var(norm_type));
            }

            //! minimum value
            Scalar min() const {
                return min_val;
            }
            //! maximum value
            Scalar max() const {
                return max_val;
            }
            Scalar range() const {
                return (max_val - min_val);
            }

            //! number of samples so far
            Scalar count() const {
                return counter.value();
            }

        private:
            stat_counter<Scalar> counter;

            Scalar r_mean;
            Scalar r_var;

            Scalar min_val;
            Scalar max_val;

            Scalar min_val_norm;
            Scalar max_val_norm;

            //! update statistics to reflect new sample (version for non-complex numbers, non-complex sample)
            void update_stats(const Scalar& sample) {
                const Scalar N = counter.value();

                if (N > Scalar(0)) {
                    if (sample < min_val) {
                        min_val = sample;
                    }

                    if (sample > max_val) {
                        max_val = sample;
                    }

                    const Scalar N_plus_1  = counter.value_plus_1();
                    const Scalar N_minus_1 = counter.value_minus_1();

                    // note: variance has to be updated before the mean

                    const Scalar tmp = sample - r_mean;

                    r_var = N_minus_1 / N * r_var + (tmp * tmp) / N_plus_1;

                    r_mean = r_mean + (sample - r_mean) / N_plus_1;
                    // r_mean = (N/N_plus_1)*r_mean + sample/N_plus_1;
                    // r_mean = (r_mean + sample/N) * N/N_plus_1;
                }
                else {
                    r_mean  = sample;
                    min_val = sample;
                    max_val = sample;

                    // r_var is initialised to zero
                    // in the constructor and reset()
                }

                counter++;
            }
        };
    }  // namespace optimisation
}  // namespace support
}  // namespace module
