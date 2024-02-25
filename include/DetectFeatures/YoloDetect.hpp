#pragma once

#include <Cameras/ImageTypes.hpp>
#include <Communication/ProcessedTypes.hpp>
#include <Cameras/ImageTypes.hpp>

int DetectYolo(const CameraImageData &InData, CameraFeatureData& OutData);

void YoloTest(VideoCaptureCameraSettings CamSett);