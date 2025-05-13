#include "Cameras/Calibfile.hpp"
#include <iostream>
#include <filesystem>
#include <set>
#include <fstream>
#include <Misc/path.hpp>
#include <nlohmann/json.hpp>
#include <Misc/MatToJSON.hpp>

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
	if (path.extension() == ".yaml")
	{
		return;
	}
	if (path.extension() == ".json")
	{
		return;
	}
	path.replace_extension(".json");
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
	if (path.extension() == ".yaml")
	{
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
	else if (path.extension() == ".json")
	{
		nlohmann::json object;
		std::ifstream file(path);
		file >> object;
		assert(object.at("Stereo") == false);
		Resolution.width = object.at("Resolution").at("width");
		Resolution.height = object.at("Resolution").at("height");
		camMatrix = JsonToMatrix<double>(object.at("Camera Matrix"));
		distCoeffs = JsonToMatrix<double>(object.at("Distortion Coefficients"));
		return true;
	}
}

void writeCameraParameters(std::filesystem::path path, Mat camMatrix, Mat distCoeffs, Size Resolution)
{
	CleanCalibrationPath(path);
	path.replace_filename(GetCalibrationFileName(path.filename()));
#if 0
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
#else
	nlohmann::json object;
	object["Camera Matrix"] = MatrixToJson<double>(camMatrix);
	object["Resolution"]["width"] = Resolution.width;
	object["Resolution"]["height"] = Resolution.height;
	object["Distortion Coefficients"] = MatrixToJson<double>(distCoeffs);
	object["Stereo"] = false;
	path.replace_extension(".json");
	ofstream file(path);
	file << object.dump(1, '\t');
#endif
}

void MigrateCameraParameters()
{
	auto path = GetCyclopsPath() / "calibration";
	for (auto i : std::filesystem::directory_iterator(path))
	{
		if(i.path().extension() == ".json")
		{
			continue;
		}
		cv::Mat camMatrix, distCoeffs;
		cv::Size resolution;
		if(readCameraParameters(i.path(), camMatrix, distCoeffs, resolution))
		{
			std::filesystem::path jsonpath = i.path();
			jsonpath.replace_extension(".json");
			writeCameraParameters(jsonpath, camMatrix, distCoeffs, resolution);
			cv::Mat camMatrix2, distCoeffs2;
			cv::Size resolution2;
			readCameraParameters(jsonpath, camMatrix2, distCoeffs2, resolution2);
			assert(cv::norm(camMatrix, camMatrix2, NORM_L1) == 0);
			assert(cv::norm(distCoeffs, distCoeffs2, NORM_L1) == 0);
			assert(resolution2 == resolution);
		}
	}
}