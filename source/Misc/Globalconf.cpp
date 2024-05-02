#include "Misc/GlobalConf.hpp"

#include <Misc/ArucoDictSize.hpp>
#include <Cameras/ImageTypes.hpp>

#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>

using namespace std;
using namespace cv;

string Scenario = "";
aruco::ArucoDetector ArucoDet;
bool HasDetector = false;
vector<UMat> MarkerImages;

bool ConfigInitialised = false;

bool RecordVideo = false;


double KeepAliveDelay = 30; //Delay between messages
double KeepAliveDuration = 3*60; //Delay before kick when no response

//Default values
CaptureConfig CaptureCfg = {(int)CameraStartType::ANY, Size(3840,3032), 1.f, 30, 1, ""};
vector<InternalCameraConfig> CamerasInternal;
CalibrationConfig CamCalConf = {40, Size(6,4), 0.5, 1.5, Size2d(4.96, 3.72)};

template<class dataType, class accessorType>
void CopyOrDefaultRef(nlohmann::json &owner, accessorType accessor, dataType &value)
{
	if (owner.contains(accessor))
	{
		value = owner[accessor];
	}
	else
	{
		owner[accessor] = value;
	}
}


template<class accessorType>
nlohmann::json& CopyOrDefaultJson(nlohmann::json &owner, accessorType accessor)
{
	if (!owner.contains(accessor))
	{
		owner[accessor] = nlohmann::json();
	}
	return owner.at(accessor);
}

void InitConfig()
{
	if (ConfigInitialised)
	{
		return;
	}
	string filepath = "../config.json";
	nlohmann::json configobj;
	try
	{
		ifstream file(filepath);
		file >> configobj;
	}
	catch(const std::exception& e)
	{
		std::cerr << "Error reading config file : " << e.what() << '\n';
	}

	CopyOrDefaultRef(configobj, "Scenario", Scenario);

	nlohmann::json &Capture = CopyOrDefaultJson(configobj, "Capture");
	{
		nlohmann::json& Resolution = CopyOrDefaultJson(Capture, "Resolution");
		CopyOrDefaultRef(Resolution, 	"Width", 			CaptureCfg.FrameSize.width);
		CopyOrDefaultRef(Resolution, 	"Height", 			CaptureCfg.FrameSize.height);
		CopyOrDefaultRef(Capture, 		"Framerate", 		CaptureCfg.CaptureFramerate);
		CopyOrDefaultRef(Capture, 		"FramerateDivider", CaptureCfg.FramerateDivider);
		CopyOrDefaultRef(Capture, 		"Method", 			CaptureCfg.StartType);
		CopyOrDefaultRef(Resolution, 	"Reduction", 		CaptureCfg.ReductionFactor);
		CopyOrDefaultRef(Capture, 		"CameraFilter", 	CaptureCfg.filter);
		
	}

	nlohmann::json &CamerasSett = CopyOrDefaultJson(configobj, "InternalCameras");
	{
		CamerasInternal.clear();
		for (size_t i = 0; i < CamerasSett.size(); i++)
		{
			CamerasInternal.push_back(InternalCameraConfig());
			CamerasInternal[i].CameraName = "GarbageFilter";
			CamerasInternal[i].LocationRelative = Affine3d::Identity().translate(Vec3d(0.1,0.2,0.3));
			CopyOrDefaultRef(CamerasSett[i], "Filter", CamerasInternal[i].CameraName);
			string Loc;
			CopyOrDefaultRef(CamerasSett[i], "Location", Loc);
			ostringstream locstream(Loc);
			//locstream >> CamerasInternal[i].LocationRelative;
		}
	}

	nlohmann::json& CalibSett = CopyOrDefaultJson(configobj, "Calibration");
	{
		CopyOrDefaultRef(CalibSett, "EdgeSize", 				CamCalConf.SquareSideLength);
		CopyOrDefaultRef(CalibSett, "NumIntersectionsX", 		CamCalConf.NumIntersections.width);
		CopyOrDefaultRef(CalibSett, "NumIntersectionsY", 		CamCalConf.NumIntersections.height);
		CopyOrDefaultRef(CalibSett, "ReprojectionErrorOffset", 	CamCalConf.ReprojectionErrorOffset);
		CopyOrDefaultRef(CalibSett, "NumImagePower", 			CamCalConf.NumImagePower);

		CopyOrDefaultRef(CalibSett, "SensorSizeX", 				CamCalConf.SensorSize.width);
		CopyOrDefaultRef(CalibSett, "SensorSizeY", 				CamCalConf.SensorSize.height);
	}

	nlohmann::json& KeepAliveSettings = CopyOrDefaultJson(configobj, "KeepAlive");
	{
		CopyOrDefaultRef(KeepAliveSettings, "Delay between queries", KeepAliveDelay);
		CopyOrDefaultRef(KeepAliveSettings, "Delay to kick", KeepAliveDuration);
	}

	try
	{
		ofstream file(filepath);
		file << std::setfill('\t') << std::setw(1);
		file << configobj;
	}
	catch(const std::exception& e)
	{
		std::cerr << "Failed to serialize config: " << e.what() << '\n';
	}
	
	
	ConfigInitialised = true;
	
}

std::filesystem::path GetAssetsPath()
{
	return std::filesystem::weakly_canonical(std::filesystem::path("../assets/"));
}

string GetScenario()
{
	InitConfig();
	return Scenario;
}

const aruco::ArucoDetector& GetArucoDetector(){
	if (!HasDetector)
	{
		auto dict = aruco::getPredefinedDictionary(aruco::DICT_4X4_100);
		auto params = aruco::DetectorParameters();
		params.cornerRefinementMethod = GetArucoReduction() == GetFrameSize() ? aruco::CORNER_REFINE_CONTOUR : aruco::CORNER_REFINE_NONE;
		params.useAruco3Detection = 0;
		//params.adaptiveThreshConstant = 20;
		double mulfac = 1.0/1.0;
		params.minMarkerPerimeterRate *= mulfac;
		params.minCornerDistanceRate *= mulfac;

		//params.minMarkerDistanceRate *= mulfac;
		auto refparams = aruco::RefineParameters();
		ArucoDet = aruco::ArucoDetector(dict, params, refparams);
	}
	return ArucoDet;
}

Size GetFrameSize()
{
	InitConfig();
	return CaptureCfg.FrameSize;
}

int GetCaptureFramerate()
{
	InitConfig();
	return (int)CaptureCfg.CaptureFramerate;
}

CameraStartType GetCaptureMethod()
{
	InitConfig();
	return (CameraStartType)CaptureCfg.StartType;
}

CaptureConfig GetCaptureConfig()
{
	InitConfig();
	return CaptureCfg;
}

float GetReductionFactor()
{
	InitConfig();
	return CaptureCfg.ReductionFactor;
}

std::pair<double, double> GetKeepAliveSettings()
{
	InitConfig();
	return {KeepAliveDelay, KeepAliveDuration};
}

Size GetArucoReduction()
{
	Size reduction;
	Size basesize = GetFrameSize();
	float reductionFactor = GetReductionFactor();
	reduction = Size(basesize.width / reductionFactor, basesize.height / reductionFactor);
	return reduction;
}

UMat& GetArucoImage(int id)
{
	if (MarkerImages.size() != ARUCO_DICT_SIZE)
	{
		MarkerImages.resize(ARUCO_DICT_SIZE);
	}
	if (MarkerImages[id].empty())
	{
		auto& det = GetArucoDetector();
		auto& dict = det.getDictionary();
		aruco::generateImageMarker(dict, id, 256, MarkerImages[id], 1);
	}
	return MarkerImages[id];
}

vector<InternalCameraConfig>& GetInternalCameraPositionsConfig()
{
	InitConfig();
	return CamerasInternal;
}

const CalibrationConfig& GetCalibrationConfig()
{
	InitConfig();
	return CamCalConf;
}