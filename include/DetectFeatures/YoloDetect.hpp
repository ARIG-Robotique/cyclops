#pragma once

#include <Cameras/ImageTypes.hpp>
#include <Communication/ProcessedTypes.hpp>

int DetectYolo(const CameraImageData &InData, CameraFeatureData& OutData);