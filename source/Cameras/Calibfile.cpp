#include "Cameras/Calibfile.hpp"
#include <iostream>
#include <filesystem>
#include <set>

using namespace std;
using namespace cv;

static set<filesystem::path> missingcalibs;

string GetCalibrationFileName(string description)
{
	string outstring;
	outstring.reserve(description.size());
	for (auto it = description.begin(); it < description.end(); it++)
	{
		switch (*it)
		{
		case '<':
		case '>':
		case ':':
		case '"':
		case '\\':
		case '|':
		case '?':
		case '*':
			break;

		case ' ':
			outstring.push_back('_');
			break;
		
		default:
			outstring.push_back(*it);
			break;
		}
	}
	return outstring;
}

void CleanCalibrationPath(std::filesystem::path &path)
{
	path = filesystem::weakly_canonical(path);
	path.replace_extension(".yaml");
}

bool readCameraParameters(std::filesystem::path path, cv::Mat &camMatrix, cv::Mat &distCoeffs, cv::Size &Resolution)
{
	CleanCalibrationPath(path);
	path.replace_filename(GetCalibrationFileName(path.filename()));
	if (!filesystem::exists(path))
	{
		if (missingcalibs.find(path) == missingcalibs.end())
		{
			missingcalibs.emplace(path);
			cerr << "Failed to read camera parameters for " << path << endl;
		}
		
		return false;
	}
	
	FileStorage fs(path, FileStorage::READ);
	if (!fs.isOpened())
	{
		//
		return false;
	}
	Mat resmat;
	fs["resolution"] >> resmat;
	Resolution.width = resmat.at<int>(0,0);
	Resolution.height = resmat.at<int>(0,1);
	Mat calibmatrix;
	fs["camera_matrix"] >> camMatrix;
	fs["distortion_coefficients"] >> distCoeffs;
	
	
	
	//cout << "Calibration file resolution = " << CalibRes << endl;
	//cout << "Reading at resolution " <<Resolution << endl;
	//cout << scalingMatrix << " * " << calibmatrix << " = " << camMatrix << endl;
	return (camMatrix.size() == Size(3,3));
}

void writeCameraParameters(std::filesystem::path path, Mat camMatrix, Mat distCoeffs, Size Resolution)
{
	CleanCalibrationPath(path);
	path.replace_filename(GetCalibrationFileName(path.filename()));
	FileStorage fs(path, FileStorage::WRITE);
	if (!fs.isOpened())
	{
		return;
	}
	Mat1i resmat(1,2);
	resmat.at<int>(0,0) = Resolution.width;
	resmat.at<int>(0,1) = Resolution.height;
	fs.write("resolution", resmat);
	fs.write("camera_matrix", camMatrix);
	fs.write("distortion_coefficients", distCoeffs);
}