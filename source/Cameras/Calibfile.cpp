#include "Cameras/Calibfile.hpp"
#include <iostream>
#include <filesystem>
#include <set>

using namespace std;
using namespace cv;

static set<string> missingcalibs;

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

bool readCameraParameters(string description, Mat& camMatrix, Mat& distCoeffs, Size Resolution)
{
	string filename = GetCalibrationFileName(description);
	auto abspath = filesystem::absolute(filename+".yaml");
	if (!filesystem::exists(abspath))
	{
		if (missingcalibs.find(filename) == missingcalibs.end())
		{
			missingcalibs.emplace(filename);
			cerr << "Failed to read camera parameters for " << filename << ".yaml" << endl;
		}
		
		return false;
	}
	
	FileStorage fs(filename + ".yaml", FileStorage::READ);
	if (!fs.isOpened())
	{
		//
		return false;
	}
	
	Mat1i resmat; Size CalibRes;
	fs["resolution"] >> resmat;
	CalibRes.width = resmat.at<int>(0,0);
	CalibRes.height = resmat.at<int>(0,1);
	Mat calibmatrix;
	fs["camera_matrix"] >> calibmatrix;
	fs["distortion_coefficients"] >> distCoeffs;
	if (Resolution.area() != 0)
	{
		Mat scalingMatrix = Mat::zeros(3,3, CV_64F);
		scalingMatrix.at<double>(0,0) = (double)Resolution.width / CalibRes.width;
		scalingMatrix.at<double>(1,1) = (double)Resolution.height / CalibRes.height;
		scalingMatrix.at<double>(2,2) = 1;
		camMatrix = scalingMatrix * calibmatrix;
	}
	else
	{
		camMatrix = calibmatrix;
	}
	
	
	
	//cout << "Calibration file resolution = " << CalibRes << endl;
	//cout << "Reading at resolution " <<Resolution << endl;
	//cout << scalingMatrix << " * " << calibmatrix << " = " << camMatrix << endl;
	return (camMatrix.size() == Size(3,3));
}

void writeCameraParameters(string description, Mat camMatrix, Mat distCoeffs, Size Resolution)
{
	string filename = GetCalibrationFileName(description);
	FileStorage fs(filename + ".yaml", FileStorage::WRITE);
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