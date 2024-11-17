#pragma once
#include <filesystem>

std::filesystem::path GetExecutablePath();
std::filesystem::path GetCyclopsPath();
std::filesystem::path GetScreenCapturePath();

void SetExecutablePath(const char* path);