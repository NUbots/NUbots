#include "DisplayTest.h"
#include "messages/PlainMessage.h"

namespace modules {
    DisplayTest::DisplayTest(NUClear::PowerPlant* plant) : Reactor(plant) {

        on<Trigger<Every<1, std::chrono::seconds>>>([this](const time_t&) {

			std::string value = "testing";
			std::cout << value << std::endl;

			auto message = std::make_unique<messages::PlainMessage>();
			message->value = value;
			emit(std::move(message));

        });
    }
}
