#pragma once

// Shared helpers used by the hwmon endpoints. Every endpoint in the original
// binary either runs a subprocess via popen() and parses its stdout, or reads
// a /proc or /sys file. This header bundles those primitives.

#include <nlohmann/json.hpp>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace limelight::hwmon {

// Run `cmd` via popen("r"), collect stdout into a string, and return it.
// Sets `exit_status_out` to the pclose() return value when non-null.
// On popen failure returns std::nullopt.
std::optional<std::string> execCapture(const std::string& cmd,
                                       int* exit_status_out = nullptr);

// Shell-free variant: runs argv[0] via execvp with literal arguments. Prefer
// this over execCapture when any argument is user-controlled.
std::optional<std::string> execCaptureArgv(const std::vector<std::string>& argv,
                                           int* exit_status_out = nullptr);

// Read the entire contents of a file into a string. Returns nullopt on failure.
std::optional<std::string> readFile(const std::string& path);

// Read first line of a file, trimmed.
std::optional<std::string> readFileLine(const std::string& path);

// Trim ASCII whitespace from both ends.
std::string trim(const std::string& s);

// Split by delimiter (single character).
std::vector<std::string> split(const std::string& s, char delim);

// Split by any-of characters, collapsing repeats.
std::vector<std::string> splitWs(const std::string& s);

// Check that a string looks like a 32-char lowercase hex boot id.
bool isValidBootId(const std::string& s);

// Turn a shell-safe string: only allow [A-Za-z0-9._@:-].
bool isSafeServiceName(const std::string& s);

// Build the standard { "success": false, "error": "<msg>" } error body.
nlohmann::json makeError(const std::string& msg);

}  // namespace limelight::hwmon
