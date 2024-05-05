#include <string>
#include <opencv2/core.hpp>
#include <filesystem>

std::string GetCalibrationFileName(std::string description);

bool readCameraParameters(std::filesystem::path path, cv::Mat &camMatrix, cv::Mat &distCoeffs, cv::Size &Resolution);

void writeCameraParameters(std::filesystem::path path, cv::Mat camMatrix, cv::Mat distCoeffs, cv::Size Resolution);