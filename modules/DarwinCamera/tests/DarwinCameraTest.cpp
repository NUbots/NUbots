#include "DarwinCamera.h"
#include "messages/Image.h"

namespace {

    class ImageReactor : public NUClear::Reactor {
    public:
        ImageReactor(NUClear::PowerPlant* plant): Reactor(plant) {
            on<Trigger<messages::Image>>([this](const messages::Image& image) {
                std::cout << "Successfully retrieved an image in ";
                std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - last).count();
                std::cout << "ms." << std::endl;
                last = std::chrono::high_resolution_clock::now();
                //this->powerPlant.shutdown();
            });

            last = std::chrono::high_resolution_clock::now();
        }
    private:
        std::chrono::high_resolution_clock::time_point last;
    };
}


int main(int argc, char *argv[]) {
    NUClear::PowerPlant::Configuration config;
    config.threadCount = 1;
    NUClear::PowerPlant plant(config);
    plant.install<modules::DarwinCamera>();
    plant.install<ImageReactor>();
    plant.start();

    return 0;
}
