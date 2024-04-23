#pragma once

#include <Cameras/ImageTypes.hpp>
#include <Communication/ProcessedTypes.hpp>
#include <Cameras/ImageTypes.hpp>

const std::string& GetYoloClassName(int index);

int GetYoloNumClasses();

int DetectYolo(const CameraImageData &InData, CameraFeatureData& OutData);

void YoloTest(VideoCaptureCameraSettings CamSett);