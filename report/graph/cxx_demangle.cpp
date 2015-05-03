#include <cxxabi.h>
#include <memory>
#include <iostream>

extern "C" {
    const char* demangle(const char* symbol);
}

const char* demangle(const char* symbol) {

    int status = -1;
    return abi::__cxa_demangle(symbol, nullptr, nullptr, &status);
}
