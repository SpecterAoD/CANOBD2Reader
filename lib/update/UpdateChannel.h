#pragma once

namespace FirmwareUpdate {

enum class UpdateChannel {
    Stable,
    Beta,
    Development
};

const char* channelName(UpdateChannel channel);
UpdateChannel parseChannel(const char* value, UpdateChannel fallback = UpdateChannel::Development);
bool channelAllows(UpdateChannel selected, UpdateChannel candidate);

} // namespace FirmwareUpdate

