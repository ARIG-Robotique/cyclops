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
#include <Misc/path.hpp>

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

void Camera::Undistort()
{
	
	#if 1
	double resolution_multiplier = Settings->UndistortResolutionMultiplier;
	#else
	double resolution_multiplier = 1;
	#endif
	Size rescaled_resolution = Size2d(Settings->Resolution)*resolution_multiplier;
	if (!HasUndistortionMaps)
	{
		//assert(Settings->IsMono());
		Size cammatsz = Settings->Lenses[0].CameraMatrix.size();
		if (cammatsz.height != 3 || cammatsz.width != 3)
		{
			RegisterError();
			cerr << "Asking for undistortion but camera matrix is invalid ! Camera " << Name << endl;
			return;
		}
		//cout << "Creating undistort map using Camera Matrix " << endl << setcopy.CameraMatrix << endl 
		//<< " and Distance coeffs " << endl << setcopy.distanceCoeffs << endl;
		LensesUndistorted.resize(Settings->Lenses.size());
		std::vector<Matx33d> R(Settings->Lenses.size());
		for (size_t i = 0; i < Settings->Lenses.size(); i++)
		{
			auto &lens = Settings->Lenses[i];
			auto &lens_undist = LensesUndistorted[i];
			lens_undist = lens;

			lens_undist.CameraMatrix = Settings->Lenses[i].CameraMatrix.clone();
			
			float focal_length_multiplier = Settings->UndistortFocalLengthDivider;
			double& cx = lens_undist.CameraMatrix.at<double>(0,2), &cy = lens_undist.CameraMatrix.at<double>(1,2);
			lens_undist.CameraMatrix.at<double>(0,0) *= resolution_multiplier/focal_length_multiplier;
			lens_undist.CameraMatrix.at<double>(1,1) *= resolution_multiplier/focal_length_multiplier;
			cx *= resolution_multiplier;
			cy *= resolution_multiplier;
			cout << "cx = " << cx << " cy = " << cy << endl;
			lens_undist.distanceCoeffs = Mat::zeros(4,1, CV_64F);

			lens_undist.ROI = Rect2i(lens.ROI.tl()*resolution_multiplier, lens.ROI.br()*resolution_multiplier);
			R[i] = Matx33d::eye();
		}
		#if 1
		if (Settings->IsStereo())
		{
			auto &lens1 = Settings->Lenses[0], &lens2 = Settings->Lenses[1];
			Affine3d Lens1ToLens2 = lens1.CameraToLens.inv() * lens2.CameraToLens;
			Affine3d Lens2ToLens1 = Lens1ToLens2.inv();
			#if 1
			Matx33d &R1 = R[0], &R2 = R[1];
			#else
			Matx33d R1, R2;
			#endif
			Affine3d &AffineToUse = Lens2ToLens1;
			auto R = AffineToUse.rotation();
			auto T = AffineToUse.translation();
			Mat &P1 = LensesUndistorted[0].CameraMatrix, &P2 = LensesUndistorted[1].CameraMatrix;
			Mat &Q = DisparityToDepth;
			cout << " C1 = " << P1 << " C2 = " << P2 <<  endl;
			stereoRectify(P1, lens1.distanceCoeffs, P2, lens2.distanceCoeffs,
				lens1.ROI.size(), R, T, R1, R2, P1, P2, Q, 
				CALIB_ZERO_DISPARITY, -1, lens1.ROI.size(), &LensesUndistorted[0].StereoROI, &LensesUndistorted[1].StereoROI);
			
			auto R_new = R2*R*R1.inv();
			auto T_new = R2*T;
			Affine3d Lens2ToLens1_new(R_new, T_new);
			LensesUndistorted[1].CameraToLens = lens1.CameraToLens*Lens2ToLens1_new.inv();

			cout << " R = " << R << " R_new = " << R_new << " T = " << T << " T_new = " << T_new << endl;

			cout << "Q = " << Q << " P1 = " << P1 << " P2 = " << P2 <<  endl;
		}
		#endif

		UndistMaps.resize(LensesUndistorted.size());
		
		for (size_t i = 0; i < Settings->Lenses.size(); i++)
		{
			auto &lens = Settings->Lenses[i];
			auto &lens_undist = LensesUndistorted[i];
			//UndistMaps[i].first = UMat(lens_undist.ROI.size(), CV_32FC1); UndistMaps[i].second = UMat(lens_undist.ROI.size(), CV_32FC1);
			Mat map1(lens_undist.ROI.size(), CV_32FC1), map2(lens_undist.ROI.size(), CV_32FC1);
			initUndistortRectifyMap(lens.CameraMatrix, lens.distanceCoeffs, R[i], 
			lens_undist.CameraMatrix, lens_undist.ROI.size(), map1.type(), map1, map2);
			map1.copyTo(UndistMaps[i].first);
			map2.copyTo(UndistMaps[i].second);
			//lens_undist.CameraToLens = Affine3d(lens_undist.CameraToLens.rotation() * R[i].inv(), lens_undist.CameraToLens.translation());
		}
		HasUndistortionMaps = true;
	}
	try
	{
		LastFrameUndistorted = UMat(rescaled_resolution, LastFrameDistorted.type());
		for (size_t i = 0; i < LensesUndistorted.size(); i++)
		{
			remap(LastFrameDistorted(Settings->Lenses[i].ROI), LastFrameUndistorted(LensesUndistorted[i].ROI), 
				UndistMaps[i].first, UndistMaps[i].second, INTER_LINEAR);
		}
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
		if (Settings->IsStereo())
		{
			frame.DisparityToDepth = DisparityToDepth;
		}
		
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
		datas.emplace_back(ObjectType::Lens, "Lens " + to_string(lensidx), Location * Settings->Lenses[lensidx].CameraToLens, LastSeenTick);
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