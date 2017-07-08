// This file is only here to define some symbols used in
// Spinnaker that aren't available in the non cxx11 abi
// They don't actually work and just make it so things compile
#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>

#if _GLIBCXX_USE_CXX11_ABI == 0

Spinnaker::CameraPtr Spinnaker::CameraList::GetBySerial(std::string) const {
    Spinnaker::CameraPtr ret{};
    return ret;
}

#endif
