#pragma once

namespace WebAssets {

/// Shared scenario list used by sender and display web pages.
/// Keeping it in one place prevents the browser UI from drifting away from the
/// simulation enum names used by the firmware.
const char* simulationScenariosJsonArray();

} // namespace WebAssets
