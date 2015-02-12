#include <cxxabi.h>
#include <iostream>
#include <memory>

inline std::string demangle(const char* symbol) {

    int status = -4; // some arbitrary value to eliminate the compiler warning
    std::unique_ptr<char, void(*)(void*)> res {
        abi::__cxa_demangle(symbol, nullptr, nullptr, &status),
        std::free
    };

    return std::string(status == 0 ? res.get() : symbol);
}

int main() {

	// While we have input
	while(!std::cin.eof()) {
		// Read from stdin
		std::string input;
		getline(std::cin, input);

		// Demangle and print the symbol
		std::cout << demangle(input.c_str()) << std::endl;
	}
}