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
	#if 0
	lenses.resize(Settings->Lenses.size());
	for (size_t i = 0; i < Settings->Lenses.size(); i++)
	{
		lenses[i].CameraMatrix = Settings->Lenses[i].CameraMatrix;
		lenses[i].ROI = Settings->Lenses[i].ROI;
		lenses[i].distanceCoeffs = Mat::zeros(4,1, CV_64F);
	}
	#else
	lenses = LensesUndistorted;
	#endif
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

vector<vector<Point2d>> AlignLensCenters(const CameraSettings &Settings)
{
	size_t num_lenses = Settings.Lenses.size();
	Point3d LensMeanCenter, LensMeanForward, LensMeanX;
	Point2d OpticalMeanCenter;
	for (size_t i = 0; i < num_lenses; i++)
	{
		auto &lens = Settings.Lenses[i];
		LensMeanCenter = Vec3d(LensMeanCenter) + lens.LensPosition.translation();
		LensMeanForward = Vec3d(LensMeanForward) + Vec3d(GetAxis(lens.LensPosition.rotation(), 2).val);
		LensMeanX = Vec3d(LensMeanX) + Vec3d(GetAxis(lens.LensPosition.rotation(), 0).val);
		OpticalMeanCenter.x += lens.CameraMatrix.at<double>(0,2);
		OpticalMeanCenter.y += lens.CameraMatrix.at<double>(1,2);
	}
	LensMeanCenter /= (double)num_lenses;
	LensMeanForward /= (double)num_lenses;
	LensMeanX /= (double)num_lenses;
	OpticalMeanCenter /= (double)num_lenses;
	Mat OptimalMatrix = Settings.Lenses[0].CameraMatrix.clone();
	OptimalMatrix.at<double>(0,0) /= Settings.UndistortFocalLengthMuliply;
	OptimalMatrix.at<double>(1,1) /= Settings.UndistortFocalLengthMuliply;
	OptimalMatrix.at<double>(0,2) = OpticalMeanCenter.x;
	OptimalMatrix.at<double>(1,2) = OpticalMeanCenter.y;
	vector<Point3d> LensFocusPoints = {LensMeanCenter + LensMeanForward*5.0, LensMeanCenter + LensMeanForward*5.0+LensMeanX};
	vector<vector<Point2d>> ConvergencePoints(num_lenses);
	for (size_t i = 0; i < num_lenses; i++)
	{
		auto &lens = Settings.Lenses[i];
		vector<Point2d> &LocalConvergencePoints = ConvergencePoints[i];
		projectPoints(LensFocusPoints, lens.LensPosition.rotation(), lens.LensPosition.translation(), 
			OptimalMatrix, Mat(), LocalConvergencePoints);
	}
	return ConvergencePoints;
}

void Camera::Undistort()
{
	
	if (!HasUndistortionMaps)
	{
		//assert(Settings->IsMono());
		double resolution_multiplier = Settings->UndistortFocalLengthMuliply;
		//double resolution_multiplier = 1;
		cv::Size rescaled_resolution = Size2d(Settings->Resolution)*resolution_multiplier;
		Mat map1(rescaled_resolution, CV_32F), map2(rescaled_resolution, CV_32F);
		Size cammatsz = Settings->Lenses[0].CameraMatrix.size();
		if (cammatsz.height != 3 || cammatsz.width != 3)
		{
			RegisterError();
			cerr << "Asking for undistortion but camera matrix is invalid ! Camera " << Name << endl;
			return;
		}
		//cout << "Creating undistort map using Camera Matrix " << endl << setcopy.CameraMatrix << endl 
		//<< " and Distance coeffs " << endl << setcopy.distanceCoeffs << endl;
		auto lens_centers = AlignLensCenters(*Settings);
		LensesUndistorted.resize(Settings->Lenses.size());
		
		for (size_t i = 0; i < Settings->Lenses.size(); i++)
		{
			auto &lens = Settings->Lenses[i];
			auto &lens_undist = LensesUndistorted[i];
			lens_undist = lens;
			lens_undist.CameraMatrix = Settings->Lenses[0].CameraMatrix.clone();
			lens_undist.CameraMatrix.at<double>(0,0) /= Settings->UndistortFocalLengthMuliply/resolution_multiplier;
			lens_undist.CameraMatrix.at<double>(1,1) /= Settings->UndistortFocalLengthMuliply/resolution_multiplier;
			lens_undist.CameraMatrix.at<double>(0,2) = lens_centers[i][0].x*resolution_multiplier;
			lens_undist.CameraMatrix.at<double>(1,2) = lens_centers[i][0].y*resolution_multiplier;
			lens_undist.distanceCoeffs = Mat::zeros(4,1, CV_64F);
			lens_undist.ROI = Rect2i(lens.ROI.tl()*resolution_multiplier, lens.ROI.br()*resolution_multiplier);
			auto submap1 = map1(lens_undist.ROI), submap2 = map2(lens_undist.ROI);
			initUndistortRectifyMap(lens.CameraMatrix, lens.distanceCoeffs, Mat::eye(3,3, CV_64F), 
			lens_undist.CameraMatrix, lens_undist.ROI.size(), CV_32FC1, submap1, submap2);
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
	std::vector<ObjectData> datas = {camera};
	for (size_t lensidx = 0; lensidx < Settings->Lenses.size(); lensidx++)
	{
		datas.emplace_back(ObjectType::Lens, "Lens " + to_string(lensidx), Location * Settings->Lenses[lensidx].LensPosition, LastSeenTick);
	}
	
	return datas;
}

void Camera::Record(filesystem::path rootPath, int RecordIdx)
{
	string folderstr = Name.substr(0, std::min<size_t>(Name.find(' '), 10));
	filesystem::create_directories(rootPath/folderstr);
	if (!RecordOutput)
	{
		RecordOutput = make_unique<VideoWriter>();
		#if 0
		cout << "VIDEOWRITER_PROP_QUALITY returned " << RecordOutput->set(VIDEOWRITER_PROP_QUALITY, 95) << endl;
		RecordOutput->open(rootPath/folderstr/"video.avi", 
		#else
		ostringstream ss;
		ss << "appsrc ! videoconvert ! x264enc bitrate=100000 speed-preset=faster " 
		<< (Settings->Lenses.size() == 2 ? "frame-packing=side-by-side " : "") 
		<< "! avimux ! filesink location=" << rootPath/folderstr/"video.avi";
		RecordOutput->open(ss.str(), CAP_GSTREAMER, 
		#endif
			cv::VideoWriter::fourcc('M', 'P', 'E', 'G'), 
			30,
			Settings->Resolution,
			GetFrame(true).Image.channels() > 1);
		cout << "Opened recording :" << RecordOutput->isOpened() <<endl;
	}
	if (!TimestampsOutput)
	{
		TimestampsOutput = make_unique<ofstream>(rootPath/folderstr/"timestamps.txt");
		writeCameraParameters(rootPath/folderstr/"calibration.json", *Settings.get());
	}
	
	#if 1
	(void) RecordIdx;
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