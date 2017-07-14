#ifndef UTILITY_VISION_SPINNAKER_H
#define UTILITY_VISION_SPINNAKER_H

#include <string>

#include <SpinGenApi/SpinnakerGenApi.h>
#include <Spinnaker.h>

#include "utility/math/comparison.h"

namespace utility {
namespace vision {

    using namespace utility::math;

    template <typename T>
    struct SpinnakerTypeMap;
    template <>
    struct SpinnakerTypeMap<int> {
        using type = Spinnaker::GenApi::CIntegerPtr;
    };
    template <>
    struct SpinnakerTypeMap<int64_t> {
        using type = Spinnaker::GenApi::CIntegerPtr;
    };
    template <>
    struct SpinnakerTypeMap<double> {
        using type = Spinnaker::GenApi::CFloatPtr;
    };

    template <typename T>
    using SpinnakerType = typename SpinnakerTypeMap<T>::type;

    template <typename T>
    inline bool getNumericParam(const Spinnaker::GenApi::INodeMap& nodeMap,
                                const std::string& param,
                                SpinnakerType<T>& value) {
        // Get a pointer to the parameter.
        SpinnakerType<T> ptr = nodeMap.GetNode(param.c_str());

        // Return the parameter if it is available.
        if (IsAvailable(ptr)) {
            value = ptr;
            return (true);
        }

        return (false);
    }

    template <typename T>
    inline bool setNumericParam(const Spinnaker::GenApi::INodeMap& nodeMap, const std::string& param, T value) {
        SpinnakerType<T> ptr;

        // Get a pointer to the parameter.
        if (getNumericParam<T>(nodeMap, param, ptr) == false) {
            return (false);
        }

        // Make sure it's writable.
        if (IsWritable(ptr)) {
            T newValue = value;

            // Ensure value is a multiple of the increment.
            // Other possibilites are no increment, and a list of accepted values.
            if (ptr->GetIncMode() == Spinnaker::GenApi::EIncMode::fixedIncrement) {
                newValue = roundUp(newValue, ptr->GetInc());
            }

            // Ensure value is within the min/max.
            newValue = clamp(ptr->GetMin(), newValue, ptr->GetMax());

            // Set the new value.
            ptr->SetValue(newValue);

            return (true);
        }

        else {
            return (false);
        }
    }

    inline bool getBooleanParam(const Spinnaker::GenApi::INodeMap& nodeMap,
                                const std::string& param,
                                Spinnaker::GenApi::CBooleanPtr& value) {
        // Get a pointer to the parameter.
        Spinnaker::GenApi::CBooleanPtr ptr = nodeMap.GetNode(param.c_str());

        // Return the parameter if it is available.
        if (IsAvailable(ptr)) {
            value = ptr;
            return (true);
        }

        return (false);
    }

    inline bool setBooleanParam(const Spinnaker::GenApi::INodeMap& nodeMap, const std::string& param, bool value) {
        Spinnaker::GenApi::CBooleanPtr ptr;

        // Get a pointer to the parameter.
        if (getBooleanParam(nodeMap, param, ptr) == false) {
            return (false);
        }

        // Make sure it's writable.
        if (IsWritable(ptr)) {
            // Set the new value.
            ptr->SetValue(value);

            return (true);
        }

        else {
            return (false);
        }
    }

    inline bool getStringParam(const Spinnaker::GenApi::INodeMap& nodeMap,
                               const std::string& param,
                               Spinnaker::GenApi::CStringPtr& value) {
        // Get a pointer to the parameter.
        Spinnaker::GenApi::CStringPtr ptr = nodeMap.GetNode(param.c_str());

        // Return the parameter if it is available.
        if (IsAvailable(ptr)) {
            value = ptr;
            return (true);
        }

        return (false);
    }

    inline bool setStringParam(const Spinnaker::GenApi::INodeMap& nodeMap,
                               const std::string& param,
                               const std::string& value) {
        Spinnaker::GenApi::CStringPtr ptr;

        // Get a pointer to the parameter.
        if (getStringParam(nodeMap, param, ptr) == false) {
            return (false);
        }

        // Make sure it's writable.
        if ((IsWritable(ptr)) && (value.length() < static_cast<size_t>(ptr->GetMaxLength()))) {
            // Set the new value.
            ptr->SetValue(value.c_str());

            return (true);
        }

        else {
            return (false);
        }
    }

    inline bool getEnumParam(const Spinnaker::GenApi::INodeMap& nodeMap,
                             const std::string& param,
                             Spinnaker::GenApi::CEnumerationPtr& value) {
        // Get a pointer to the parameter.
        Spinnaker::GenApi::CEnumerationPtr ptr = nodeMap.GetNode(param.c_str());

        // Return the parameter if it is available.
        if (IsAvailable(ptr)) {
            value = ptr;
            return (true);
        }

        return (false);
    }

    inline bool getEnumEntry(const Spinnaker::GenApi::CEnumerationPtr& enumName,
                             const std::string& enumEntry,
                             Spinnaker::GenApi::CEnumEntryPtr& value) {
        // Get a pointer to the parameter.
        Spinnaker::GenApi::CEnumEntryPtr ptr = enumName->GetEntryByName(enumEntry.c_str());

        // Return the parameter if it is available.
        if (IsAvailable(ptr)) {
            value = ptr;
            return (true);
        }

        return (false);
    }

    inline bool setEnumParam(const Spinnaker::GenApi::INodeMap& nodeMap,
                             const std::string& enumName,
                             const std::string& enumEntry) {
        Spinnaker::GenApi::CEnumerationPtr pEnum;

        // Get a pointer to the parameter.
        if (getEnumParam(nodeMap, enumName, pEnum) == false) {
            return (false);
        }

        // Make sure it's writable.
        if (IsWritable(pEnum)) {
            Spinnaker::GenApi::CEnumEntryPtr pEntry;

            if (getEnumEntry(pEnum, enumEntry, pEntry) == false) {
                return (false);
            }

            if (IsReadable(pEntry)) {
                // Set the new value.
                pEnum->SetIntValue(pEntry->GetValue());
            }

            else {
                return (false);
            }

            return (true);
        }

        else {
            return (false);
        }
    }
}
}

#endif  // UTILITY_VISION_SPINNAKER_H
