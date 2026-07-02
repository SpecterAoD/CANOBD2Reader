#pragma once

#include <cstddef>
#include <cstdint>

#include "IsoTpHandler.h"
#include "UdsTypes.h"

namespace Uds {

class Client {
public:
    void setRequestId(uint32_t requestId) { isoTp_.setRequestId(requestId); }
    uint32_t requestId() const { return isoTp_.requestId(); }

    bool requestDefaultSession(Response& response);
    bool testerPresent(Response& response);
    bool readDataByIdentifier(uint16_t did, Response& response);
    bool readDtcInformation(uint8_t subFunction, uint8_t statusMask, Response& response);

    static bool isServiceAllowed(Service service);

private:
    bool request(Service service,
                 const uint8_t* payload,
                 std::size_t payloadLength,
                 Response& response);

    IsoTp::IsoTpHandler isoTp_{};
};

} // namespace Uds
