#include "DarwinCamera.h"
#include "messages/NUImage.h"

namespace {
	class ImageReactor : public NUClear::Reactor {
	public:
		ImageReactor(NUClear::PowerPlant& plant): Reactor(plant) {
			on<Trigger<messages::NUImage>>([this](const messages::NUImage& image) {
				std::cout << "Successfully retrieved an image." << std::endl;
				this->powerPlant.shutdown();
			});
		}
	};
}


int main(int argc, char *argv[]) {
    NUClear::PowerPlant::Configuration config;
    config.threadCount = 1;
    NUClear::PowerPlant plant(config);
	plant.install<modules::DarwinCamera>();
    plant.install<ImageReactor>();
    
    return 0;
}
