#include "Cameras/Camera.hpp"

#include <iostream> // for standard I/O
#include <math.h>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <Misc/math2d.hpp>
#include <Misc/math3d.hpp>

#include <Cameras/Calibfile.hpp>

#include <ArucoPipeline/ObjectTracker.hpp>
#include <Misc/GlobalConf.hpp>

using namespace cv;
using namespace std;

void Camera::RegisterError()
{
	errors += 10;
}

void Camera::RegisterNoError()
{
	errors = std::max(0, errors -1);
}

const CameraSettings* Camera::GetCameraSettings() const
{
	return Settings.get();
}

bool Camera::SetCameraSetting(std::shared_ptr<CameraSettings> InSettings)
{
	if (connected)
	{
		cerr << "WARNING: Tried to set camera settings, but camera is already started ! Settings have not been applied." << endl;
		return false;
	}
	Settings = InSettings;
	return true;
}

bool Camera::SetCalibrationSetting(Mat CameraMatrix, Mat DistanceCoefficients)
{
	Settings->CameraMatrix = CameraMatrix;
	Settings->distanceCoeffs = DistanceCoefficients;
	HasUndistortionMaps = false;
	return true;
}

void Camera::GetCameraSettingsAfterUndistortion(Mat& CameraMatrix, Mat& DistanceCoefficients) const
{
	CameraMatrix = Settings->CameraMatrix;
	//DistanceCoefficients = Settings.distanceCoeffs; //FIXME
	DistanceCoefficients = Mat::zeros(4,1, CV_64F);
}

bool Camera::StartFeed()
{
	cerr << "ERROR : Tried to start feed on base class Camera !" << endl;
	return false;
}

void Camera::SetPositionLock(bool state)
{
	if (state == PositionLocked)
	{
		return;
	}
	
	PositionLocked = state;

	cout << "Camera " << Name << " is now " << (PositionLocked ? "LOCKED" : "Unlocked") << endl;
}

void Camera::UpdateFrameNumber()
{
	FrameNumber++;
	for (size_t i = 0; i < Settings->CameraLockToggles.size(); i++)
	{
		if (FrameNumber == (int)Settings->CameraLockToggles[i])
		{
			SetPositionLock(!PositionLocked);
		}
	}
}

bool Camera::Grab()
{
	if (!connected)
	{
		return false;
	}
	
	UpdateFrameNumber();
	captureTime = std::chrono::steady_clock::now();
	grabbed = true;
	return false;
}

bool Camera::Read()
{
	if (!connected)
	{
		return false;
	}
	if (!grabbed)
	{
		UpdateFrameNumber();
		captureTime = std::chrono::steady_clock::now();
	}
	grabbed = false;
	return false;
}

void Camera::Undistort()
{
	
	if (!HasUndistortionMaps)
	{
		Size cammatsz = Settings->CameraMatrix.size();
		if (cammatsz.height != 3 || cammatsz.width != 3)
		{
			RegisterError();
			cerr << "Asking for undistortion but camera matrix is invalid ! Camera " << Name << endl;
			return;
		}
		//cout << "Creating undistort map using Camera Matrix " << endl << setcopy.CameraMatrix << endl 
		//<< " and Distance coeffs " << endl << setcopy.distanceCoeffs << endl;
		Mat map1, map2;

		initUndistortRectifyMap(Settings->CameraMatrix, Settings->distanceCoeffs, Mat::eye(3,3, CV_64F), 
		Settings->CameraMatrix, Settings->Resolution, CV_32FC1, map1, map2);
		map1.copyTo(UndistMap1);
		map2.copyTo(UndistMap2);
		HasUndistortionMaps = true;
	}
	try
	{
		remap(LastFrameDistorted, LastFrameUndistorted, UndistMap1, UndistMap2, INTER_LINEAR);
	}
	catch(const std::exception& e)
	{
		std::cerr << "Camera undistort failed : " << e.what() << '\n';
		return;
	}
}

CameraImageData Camera::GetFrame(bool Distorted) const
{
	CameraImageData frame;
	frame.Distorted = Distorted;
	frame.CameraName = Name;
	if (Distorted)
	{
		frame.CameraMatrix = Settings->CameraMatrix;
		frame.DistanceCoefficients = Settings->distanceCoeffs;
		frame.Image = LastFrameDistorted;
	}
	else
	{
		GetCameraSettingsAfterUndistortion(frame.CameraMatrix, frame.DistanceCoefficients);
		frame.Image = LastFrameUndistorted;
	}
	frame.GrabTime = captureTime;
	frame.Valid = true;
	return frame;
}

vector<ObjectData> Camera::ToObjectData() const
{
	ObjectData camera(ObjectType::Camera, Name, Location, LastSeenTick);
	return {camera};
}

void Camera::Record(filesystem::path rootPath, int RecordIdx)
{
	string folderstr = Name.substr(0, std::min<size_t>(Name.find(' '), 10));
	char buffer[16]= {0};
	snprintf(buffer, sizeof(buffer)-1, "%04d", RecordIdx);
	auto writepath = rootPath/folderstr/(string(buffer) + string(".jpg"));
	filesystem::create_directories(writepath.parent_path());
	imwrite(writepath.string(), GetFrame(true).Image);
}