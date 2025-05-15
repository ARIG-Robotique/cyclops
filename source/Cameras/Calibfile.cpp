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
	CameraSettings sett;
	bool retval = readCameraParameters(path, sett);
	if (!retval)
	{
		return false;
	}
	
	assert(sett.IsMono());
	camMatrix = sett.Lenses[0].CameraMatrix;
	distCoeffs = sett.Lenses[0].distanceCoeffs;
	Resolution = sett.Resolution;
	return retval;
}

void writeCameraParameters(std::filesystem::path path, Mat camMatrix, Mat distCoeffs, Size Resolution)
{
	CameraSettings sett;
	sett.Lenses.resize(1);
	sett.Lenses[0].CameraMatrix = camMatrix;
	sett.Lenses[0].distanceCoeffs = distCoeffs;
	sett.Lenses[0].ROI = Rect2i(Point2i(0,0), Resolution);
	sett.Resolution = Resolution;
	writeCameraParameters(path, sett);
}

bool readCameraParameters(std::filesystem::path path, CameraSettings &Settings)
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
		Settings.Resolution.width = resmat.at<int>(0,0);
		Settings.Resolution.height = resmat.at<int>(0,1);
		Mat calibmatrix;
		Settings.Lenses.resize(1);
		fs["camera_matrix"] >> Settings.Lenses[0].CameraMatrix;
		fs["distortion_coefficients"] >> Settings.Lenses[0].distanceCoeffs;
		
		//cout << "Calibration file resolution = " << CalibRes << endl;
		//cout << "Reading at resolution " <<Resolution << endl;
		//cout << scalingMatrix << " * " << calibmatrix << " = " << camMatrix << endl;
		return (Settings.Lenses[0].CameraMatrix.size() == Size(3,3));
	}
	else if (path.extension() == ".json")
	{
		nlohmann::json object;
		std::ifstream file(path);
		file >> object;
		cv::Size current_resolution = JsonToSize<int>(object.at("Current Resolution"));
		for (auto &&calibration : object.at("Calibrations"))
		{
			cv::Size stored_resolution = JsonToSize<int>(calibration.at("Resolution"));
			if (current_resolution != stored_resolution)
			{
				continue;
			}
			
			for (auto &&lens_json : calibration.at("Lenses"))
			{
				LensSettings lens_struct;
				lens_struct.CameraMatrix = JsonToMatrix<double>(lens_json.at("Camera Matrix"));
				lens_struct.distanceCoeffs = JsonToMatrix<double>(lens_json.at("Distortion Coefficients"));
				lens_struct.ROI = JsonToRect<int>(lens_json.at("ROI"));
			}
		}
		
		
		return true;
	}
}

void writeCameraParameters(std::filesystem::path path, const CameraSettings &Settings)
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
	nlohmann::json object, calibration;
	object["Current Resolution"] = SizeToJson<int>(Settings.Resolution);
	calibration["Resolution"] = SizeToJson<int>(Settings.Resolution);
	auto lenses = calibration["Lenses"];
	for (size_t i = 0; i < Settings.Lenses.size(); i++)	
	{
		lenses[i]["Camera Matrix"] = MatrixToJson<double>(Settings.Lenses[i].CameraMatrix);
		lenses[i]["Distortion Coefficients"] = MatrixToJson<double>(Settings.Lenses[i].distanceCoeffs);
		lenses[i]["ROI"] = RectToJson<int>(Settings.Lenses[i].ROI);
	}
	object["Calibrations"][0] = calibration;
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