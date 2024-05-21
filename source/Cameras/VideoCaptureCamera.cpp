#include "Cameras/VideoCaptureCamera.hpp"


#include <iostream> // for standard I/O
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion
#include <stdlib.h>
#include <filesystem>
#include <thread>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <thirdparty/list-devices.hpp>
#include <thirdparty/serialib.h>

#include <Cameras/Calibfile.hpp>
#include <Misc/FrameCounter.hpp>

#include <ArucoPipeline/TrackedObject.hpp> //CameraView
#include <ArucoPipeline/ObjectTracker.hpp>
#include <Misc/GlobalConf.hpp>

using namespace cv;
using namespace std;

//#define USE_VIDEO_RECORDING

bool VideoCaptureCamera::StartFeed()
{
	auto globalconf = GetCaptureConfig();
	if (connected)
	{
		return false;
	}
	grabbed = false;

	VideoCaptureCameraSettings* Settingscast = dynamic_cast<VideoCaptureCameraSettings*>(Settings.get());

	string pathtodevice = Settingscast->DeviceInfo.device_paths[0];	
	Name = Settingscast->DeviceInfo.device_description + string(" @ ") +  pathtodevice;
	
	ostringstream sizestream;
	sizestream << "width=(int)" << Settings->Resolution.width
			<< ", height=(int)" << Settings->Resolution.height;
	switch (Settingscast->StartType)
	{
	case CameraStartType::GSTREAMER_CPU:
		{
			ostringstream capnamestream;
			capnamestream << "v4l2src device=" << pathtodevice << " io-mode=0 ! image/jpeg, width=" 
			<< Settings->Resolution.width << ", height=" << Settings->Resolution.height << ", framerate="
			<< (int)Settings->Framerate << "/" << (int)Settings->FramerateDivider << " ! ";
			if (Settingscast->StartType == CameraStartType::GSTREAMER_CPU)
			{
				capnamestream << "jpegdec ! videoconvert ! ";
			}
			capnamestream << "video/x-raw, format=BGR ! ";
			capnamestream << "appsink drop=1";
			Settingscast->StartPath = capnamestream.str();
			Settingscast->ApiID = CAP_GSTREAMER;
		}
		break;
	case CameraStartType::PLAYBACK:
		cout << "Starting camera @" << pathtodevice << " in playback mode (file " << Settingscast->StartPath << ")" << endl;
		Settingscast->StartPath = filesystem::weakly_canonical(Settingscast->StartPath);
		Settingscast->ApiID = CAP_ANY;
		break;
	default:
		cerr << "WARNING : Unrecognised Camera Start Type in VideoCaptureCamera, defaulting to auto API" << endl;
		[[fallthrough]];
	case CameraStartType::ANY:
		Settingscast->StartPath = pathtodevice;
		Settingscast->ApiID = CAP_ANY;
		break;
	}
	//char commandbuffer[1024];
	//snprintf(commandbuffer, sizeof(commandbuffer), "v4l2-ctl -d %s -c exposure_auto=%d,exposure_absolute=%d", pathtodevice.c_str(), 1, 32);
	//cout << "Aperture system command : " << commandbuffer << endl;
	//system(commandbuffer);
	feed = make_unique<VideoCapture>();
	cout << "Opening device at \"" << Settingscast->StartPath << "\" with API id " << Settingscast->ApiID << endl;
	feed->open(Settingscast->StartPath, Settingscast->ApiID);
	if (Settingscast->StartType == CameraStartType::ANY)
	{
		feed->set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
		//feed->set(CAP_PROP_FOURCC, VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
		feed->set(CAP_PROP_FRAME_WIDTH, Settings->Resolution.width);
		feed->set(CAP_PROP_FRAME_HEIGHT, Settings->Resolution.height);
		feed->set(CAP_PROP_FPS, Settings->Framerate/Settings->FramerateDivider);
		//feed->set(CAP_PROP_AUTO_EXPOSURE, 3) ;
		//feed->set(CAP_PROP_EXPOSURE, 300) ;
		feed->set(CAP_PROP_BUFFERSIZE, 1);

		Settingscast->Resolution.width = feed->get(CAP_PROP_FRAME_WIDTH);
		Settingscast->Resolution.height = feed->get(CAP_PROP_FRAME_HEIGHT);
	}
	
	connected = true;
	
	return true;
}

bool VideoCaptureCamera::Grab()
{
	if (!connected)
	{
		return false;
	}
	bool grabsuccess = false;
	grabsuccess = feed->grab();
	if (grabsuccess)
	{
		grabbed = true;
		captureTime = std::chrono::steady_clock::now();
		RegisterNoError();
	}
	else
	{
		cerr << "Failed to grab frame for camera " << Name <<endl;
		grabbed = false;
		RegisterError();
	}
	
	return grabsuccess;
}

bool VideoCaptureCamera::Read()
{
	if (!connected)
	{
		return false;
	}
	bool ReadSuccess = false;
	bool HadGrabbed = grabbed;
	grabbed = false;
	LastFrameDistorted = UMat();
	LastFrameUndistorted = UMat();
	if (HadGrabbed)
	{
		ReadSuccess = feed->retrieve(LastFrameDistorted);
	}
	else
	{
		ReadSuccess = feed->read(LastFrameDistorted);
		captureTime = std::chrono::steady_clock::now();
	}
	
	if (!ReadSuccess)
	{
		cerr << "Failed to read frame for camera " << Name <<endl;
		RegisterError();
		return false;
	}
	RegisterNoError();
	
	return true;
}