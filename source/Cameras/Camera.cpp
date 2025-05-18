#include "Cameras/Camera.hpp"

#include <iostream> // for standard I/O
#include <fstream>
#include <math.h>
#include <cassert>

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

bool Camera::SetLensSetting(std::vector<LensSettings> lenses)
{
	Settings->Lenses = lenses;
	HasUndistortionMaps = false;
	return true;
}

void Camera::GetCameraSettingsAfterUndistortion(std::vector<LensSettings> &lenses) const
{
	lenses.resize(Settings->Lenses.size());
	for (size_t i = 0; i < Settings->Lenses.size(); i++)
	{
		lenses[i].CameraMatrix = Settings->Lenses[i].CameraMatrix;
		lenses[i].ROI = Settings->Lenses[i].ROI;
		lenses[i].distanceCoeffs = Mat::zeros(4,1, CV_64F);
	}
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
		if (FrameNumber == Settings->CameraLockToggles[i])
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
		//assert(Settings->IsMono());
		Mat map1(Settings->Resolution, CV_32F), map2(Settings->Resolution, CV_32F);
		Size cammatsz = Settings->Lenses[0].CameraMatrix.size();
		if (cammatsz.height != 3 || cammatsz.width != 3)
		{
			RegisterError();
			cerr << "Asking for undistortion but camera matrix is invalid ! Camera " << Name << endl;
			return;
		}
		//cout << "Creating undistort map using Camera Matrix " << endl << setcopy.CameraMatrix << endl 
		//<< " and Distance coeffs " << endl << setcopy.distanceCoeffs << endl;
		for (size_t i = 0; i < Settings->Lenses.size(); i++)
		{
			auto &lens = Settings->Lenses[i];
			initUndistortRectifyMap(lens.CameraMatrix, lens.distanceCoeffs, Mat::eye(3,3, CV_64F), 
			lens.CameraMatrix, lens.ROI.size(), CV_32FC1, map1(lens.ROI), map2(lens.ROI));
		}
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
	//assert(Settings->IsMono() || Distorted);
	CameraImageData frame;
	frame.Distorted = Distorted;
	frame.CameraName = Name;
	if (Distorted)
	{
		frame.lenses = Settings->Lenses;
		frame.Image = LastFrameDistorted;
	}
	else
	{
		GetCameraSettingsAfterUndistortion(frame.lenses);
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
	filesystem::create_directories(rootPath/folderstr);
	if (!RecordOutput)
	{
		RecordOutput = make_unique<VideoWriter>(rootPath/folderstr/"video.avi", 
			cv::VideoWriter::fourcc('H', '2', '6', '4'), 
			Settings->Framerate/(double)Settings->FramerateDivider,
			Settings->Resolution);
		cout << "Opened recording :" << RecordOutput->isOpened() <<endl;
	}
	if (!TimestampsOutput)
	{
		TimestampsOutput = make_unique<ofstream>(rootPath/folderstr/"timestamps.txt");
		writeCameraParameters(rootPath/folderstr/"calibration.json", *Settings.get());
	}
	
	#if 1
	if (RecordOutput)
	{
		auto image = GetFrame(true);
		RecordOutput->write(image.Image);
		if (TimestampsOutput)
		{
			if (!record_start.has_value())
			{
				record_start = image.GrabTime;
			}
			auto delta = (image.GrabTime - record_start.value());
			*TimestampsOutput.get() << delta.count() << endl;
		}
		
	}

	
	#else
	char buffer[16]= {0};
	snprintf(buffer, sizeof(buffer)-1, "%04d", RecordIdx);
	auto writepath = rootPath/folderstr/(string(buffer) + string(".jpg"));
	
	imwrite(writepath.string(), GetFrame(true).Image);
	#endif
}