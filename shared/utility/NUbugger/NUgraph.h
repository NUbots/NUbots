#ifndef NUGRAPH_H
#define    NUGRAPH_H

#include <nuclear>
#include "messages/NUbuggerDataPoint.h"

using messages::NUbugger::DataPoint;

namespace utility {
namespace NUbugger{

    template<typename... Values>
    inline std::unique_ptr<DataPoint> graph(std::string label, Values... values) {
        return std::make_unique<DataPoint>(DataPoint{
            label, {values...}
        });
    }

}
}

#endif
