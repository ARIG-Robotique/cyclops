#include <iostream> // for standard I/O
#include <string>   // for strings
#include <sstream>  // string to number conversion
#include <math.h>

#include <thread>
#include <filesystem>

#include <opencv2/core.hpp>     // Basic OpenCV structures (Mat, Scalar)
#include <opencv2/core/ocl.hpp> //opencl
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>


#include <GlobalConf.hpp>
#include <Cameras/VideoCaptureCamera.hpp>
#include <Cameras/CameraManager.hpp>
#include <ArucoPipeline/ObjectTracker.hpp>
#include <Calibrate.hpp>
#include <Visualisation/BoardGL.hpp>
#include <Visualisation/ImguiWindow.hpp>
#include <EntryPoints/CDFRExternal.hpp>
#include <EntryPoints/CDFRInternal.hpp>
#include <EntryPoints/Mapping.hpp>

#include <Communication/AdvertiseMV.hpp>
#include <Communication/TCPJsonHost.hpp>

#include <stdlib.h> //env vars

#ifdef WITH_X11
#include <X11/Xlib.h>
#endif

using namespace std;
using namespace cv;

void ConfigureOpenCL(bool enable)
{
	ocl::setUseOpenCL(enable);

	if (!enable)
	{
		return;
	}
	

	if (!ocl::haveOpenCL())
	{
		cerr << "OpenCL is not available..." << endl;
		return;
	}

	ocl::Context context;
	if (!context.create(ocl::Device::TYPE_ALL))
	{
		cerr << "Failed creating the context..." << endl;
		return;
	}

	cout << context.ndevices() << " OpenCL devices detected." << endl; //This bit provides an overview of the OpenCL devices you have in your computer
	for (int i = 0; i < context.ndevices(); i++)
	{
		ocl::Device device = context.device(i);
		cout << "\tname:              " << device.name() << endl;
		cout << "\tavailable:         " << device.available() << endl;
		cout << "\timageSupport:      " << device.imageSupport() << endl;
		cout << "\tOpenCL_C_Version:  " << device.OpenCL_C_Version() << endl;
	}
}

int main(int argc, char** argv )
{
	const string keys = 
		"{help h usage ? |      | print this message}"
		"{direct d       |      | show direct camera output}"
		"{build b        |      | print build information}"
		"{calibrate c    |      | start camera calibration wizard}"
		"{marker m       |      | print out markers}"
		"{opengl ogl     |      | runs opengl test}"
		"{server s       |      | force server/client state}"
		"{map            |      | runs object mapping, using saved images and calibration}"
		"{nodisplay | | start without a display}"
		;
	CommandLineParser parser(argc, argv, keys);

	if (parser.has("help"))
	{
		parser.printMessage();
		exit(EXIT_SUCCESS);
	}
	if (parser.has("build"))
	{
		cout << getBuildInformation() << endl;
		exit(EXIT_SUCCESS);
	}
	
	bool nodisplay = parser.has("nodisplay");


	//putenv("GST_DEBUG=jpegdec:4"); //enable gstreamer debug
	
	ConfigureOpenCL(true);

	#ifdef WITH_X11
	if (nodisplay)
	{
		SetNoScreen();
	}
	else
	{
		XInitThreads();
		cout << "Detected screen resolution : " << GetScreenResolution() << endl;
		cout << "Detected screen size : " << GetScreenSize() << endl;
	}
	#endif

	
	if (parser.has("marker"))
	{
		auto& detector = GetArucoDetector();
		auto& dictionary = detector.getDictionary();
		filesystem::create_directory("../markers");
		for (int i = 0; i < ARUCO_DICT_SIZE; i++)
		{
			UMat markerImage;
			aruco::generateImageMarker(dictionary, i, 1024, markerImage, 1);
			char buffer[30];
			snprintf(buffer, sizeof(buffer)/sizeof(char), "../markers/marker%d.png", i);
			imwrite(buffer, markerImage);
		}
		exit(EXIT_SUCCESS);
	}
	
	vector<VideoCaptureCameraSettings> CamSett = CameraManager::autoDetectCameras(GetCaptureMethod(), GetCaptureConfig().filter, "", false);

	if (CamSett.size() == 0)
	{
		cerr << "No cameras detected" << endl;
	}
	
	
	if (parser.has("calibrate"))
	{
		cout << "Starting calibration of camera index" << parser.get<int>("calibrate") <<endl;
		int camIndex = parser.get<int>("calibrate");
		if (0<= camIndex && camIndex < CamSett.size())
		{
			docalibration(CamSett[camIndex]);
			exit(EXIT_SUCCESS);
		}
		else
		{
			docalibration(VideoCaptureCameraSettings());
			exit(EXIT_SUCCESS);
		}
		
		exit(EXIT_FAILURE);
		
	}

	bool direct = parser.has("direct");
	bool opengl = !parser.has("nodisplay");
	
	if (parser.has("map"))
	{
		MappingSolve();
		exit(EXIT_SUCCESS);
	}
	
	//AdvertiseMV advertiser;
	TCPJsonHost JsonHost(50667);
	
	CDFRExternal ExternalCameraHost(direct, opengl);

	while (!ExternalCameraHost.IsKilled() && !JsonHost.IsKilled())
	{
		this_thread::sleep_for(chrono::milliseconds(10));
	}
	
	// the camera will be deinitialized automatically in VideoCapture destructor
	return 0;
}