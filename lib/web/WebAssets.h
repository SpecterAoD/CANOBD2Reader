#pragma once

// Static web assets shared by sender and display.
namespace WebAssets {

// Scenario names for the simulation selector. Must match Simulation::Scenario.
const char* simulationScenariosJsonArray();

// GitHub update / WiFi / firmware upload page. "%%DEVICE%%" is replaced by
// the device name. All dynamic values are inserted via textContent or an
// escaping helper, never as raw HTML.
const char* githubUpdatePage();
const char* senderPage();
const char* displayPage();

// Small JS helper block (esc(), api()) included by the device main pages.
const char* commonScript();

} // namespace WebAssets
