#include <string>
#include <opencv2/core.hpp>
#include <filesystem>
#include <Cameras/ImageTypes.hpp>

std::string GetCalibrationFileName(std::string description);

bool readCameraParameters(std::filesystem::path path, cv::Mat &camMatrix, cv::Mat &distCoeffs, cv::Size &Resolution);

void writeCameraParameters(std::filesystem::path path, cv::Mat camMatrix, cv::Mat distCoeffs, cv::Size Resolution);

bool readCameraParameters(std::filesystem::path path, CameraSettings &Settings);

void writeCameraParameters(std::filesystem::path path, const CameraSettings &Settings);

void MigrateCameraParameters();