#include "CANDecoder.h"

namespace CANDecoder {

DecodedFrame decode(const twai_message_t& message) {
    DecodedFrame decoded;

    size_t offset = snprintf(decoded.raw, sizeof(decoded.raw),
                             "0x%03lX DLC%u",
                             static_cast<unsigned long>(message.identifier),
                             static_cast<unsigned int>(message.data_length_code));

    for (uint8_t i = 0; i < message.data_length_code && offset < sizeof(decoded.raw); ++i) {
        offset += snprintf(decoded.raw + offset, sizeof(decoded.raw) - offset,
                           " %02X", message.data[i]);
    }

    if (message.identifier >= 0x7E8 && message.identifier <= 0x7EF &&
        message.data_length_code >= 3 && message.data[1] >= 0x40) {
        snprintf(decoded.hint, sizeof(decoded.hint), "OBD Antwort Mode %02X PID %02X",
                 message.data[1] - 0x40, message.data[2]);
    } else if (message.identifier == 0x7DF) {
        snprintf(decoded.hint, sizeof(decoded.hint), "OBD Anfrage Broadcast");
    } else {
        snprintf(decoded.hint, sizeof(decoded.hint), "CAN Rohdaten");
    }

    return decoded;
}

} // namespace CANDecoder
