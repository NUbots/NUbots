#include "DabsonTest.h"


#include "utility/NUbugger/NUgraph.h"
using utility::NUbugger::graph;

namespace modules {
    namespace behaviours {
        namespace tools {

            DabsonTest::DabsonTest(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)), k(1.001) {
                
                // Get the scripts to run from the command line
                on<Trigger<Every<10, std::chrono::milliseconds>>, Options<Sync<DabsonTest>>>([this](const time_t& time) {
                    
                    double freq = 0.25;

                    double t = time.time_since_epoch().count() / double(NUClear::clock::period::den);

                    float v = 2 * M_PI * freq * cos(2 * M_PI * freq * t);
                    
                    emit(graph("Velocity", v));

                    arma::vec vm = { v };
                    
                    arma::mat pn = arma::zeros(1, 1);
                    
                    k.timeUpdate(0.01, pn, vm);
                    
                    emit(graph("Kalman", k.get()[0]));
                    emit(graph("Kalman Confidence", k.getCovariance()[0]));
                });
                
                on<Trigger<Every<100, Per<std::chrono::seconds>>>, Options<Sync<DabsonTest>>>([this](const time_t& time){
                    
                    double t = time.time_since_epoch().count() / double(NUClear::clock::period::den);
                    
                    double freq1 = 0.25;
                    double freq2 = 0.025;
                    
                    float s = sin(2 * M_PI * freq1 * t);
                    float offset = 10 * sin(2 * M_PI * freq2 * t);
                    
                    s += (rand()/double(RAND_MAX) - 0.5) * 0.4;
                    s += offset;
                    
                    arma::vec pm = { s };
                    
                    k.measurementUpdate(pm, 0.2 * arma::ones(1, 1), arma::mat());
                    
                    
                    emit(graph("Sin", s));
                    
                });
            }
            
        }  // tools
    }  // behaviours
}  // modules
