#include "WebAssets.h"

namespace WebAssets {

const char* simulationScenariosJsonArray() {
    return "['NormalSingleFrame','NormalMultiFrameVin','NormalMultiFrameDtc',"
           "'FlowControlRequired','TimeoutAfterFirstFrame','SequenceError',"
           "'BufferOverflow','MultipleEcusResponse','NegativeResponse',"
           "'DisplayNormalValues','DisplayWarningValues','DisplayCriticalValues',"
           "'DisplayTimeoutValues','DisplayMixedValues']";
}

} // namespace WebAssets
