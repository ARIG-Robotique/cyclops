#pragma once
#include <filesystem>

std::filesystem::path GetExecutablePath();
std::filesystem::path GetCyclopsPath();

void SetExecutablePath(const char* path);