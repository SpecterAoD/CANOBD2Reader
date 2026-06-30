#pragma once

#include <array>
#include <cstddef>
#include "IsoTpTypes.h"
#include "SimulationTypes.h"

namespace Simulation {

struct IsoTpSimulationSequence {
    std::array<IsoTp::CanFrame, 8> frames{};
    std::size_t frameCount = 0;
    bool flowControlExpected = false;
    bool timeoutExpected = false;
    bool negativeResponse = false;
};

IsoTpSimulationSequence buildIsoTpSequence(Scenario scenario);

}
