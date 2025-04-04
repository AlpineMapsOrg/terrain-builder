#pragma once

#include <filesystem>

namespace fs = std::filesystem;

bool double_eq(double a, double b, double eps);

// Takes a path relative to fixture directory and returns an absolute file path
fs::path fixture_path(const fs::path& fragment);
