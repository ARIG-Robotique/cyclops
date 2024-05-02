#include "Communication/JsonListener.hpp"
#include <Communication/Transport/TCPTransport.hpp>
#include <EntryPoints/CDFRExternal.hpp>
#include <EntryPoints/CDFRInternal.hpp>
#include <Communication/TCPJsonHost.hpp>
#include <Misc/math3d.hpp>
#include <Misc/math2d.hpp>
#include <Misc/GlobalConf.hpp>

#include <opencv2/imgcodecs.hpp>
#include <libbase64.h>

#include <nlohmann/json.hpp>
#include <iostream>
#include <set>

using namespace std;
using namespace nlohmann;

JsonListener::JsonListener(TCPTransport* InTransport, string InClientName, TCPJsonHost* InParent)
	:Transport(InTransport), ClientName(InClientName), Parent(InParent)
{
	LastAliveSent = chrono::steady_clock::now();
	LastAliveReceived = LastAliveSent;
	if (ListenThread)
	{
		return;
	}
	ListenThread = make_unique<thread>(&JsonListener::ThreadEntryPoint, this);
	cout << "Json listen thread for " << ClientName << " started" << endl;
}

JsonListener::~JsonListener()
{
	killed = true;
	if (ListenThread)
	{
		ListenThread->join();
	}
	cout << "Json listen thread for " << ClientName << " stoppped" << endl;
}

string JsonListener::JavaCapitalize(string source)
{
	for (auto i = source.begin(); i != source.end(); i++)
	{
		if (*i == ' ')
		{
			*i = '_';
		}
		else if (*i >= 'a' && *i <= 'z')
		{
			*i += 'A'-'a';
		}
	}
	return source;
}

json JsonListener::ObjectToJson(const ObjectData& Object)
{
	json objectified;
	objectified["type"] = JavaCapitalize(ObjectTypeNames.at(Object.type));
	objectified["name"] = JavaCapitalize(Object.name);
	if (Object.metadata.size() > 0)
	{
		objectified["meta"] = Object.metadata;
	}
	
	bool requireCoord = Object.type != ObjectType::SolarPanel;
	double rotZ = GetRotZ(Object.location.rotation());
	double rotZdeg = rotZ*180.0/M_PI;
	switch (ObjectMode)
	{
	case TransformMode::Float2D:
		if (requireCoord)
		{
			objectified["x"] = Object.location.translation()[0];
			objectified["y"] = Object.location.translation()[1];
		}
		objectified["r"] = rotZ;
		break;
	case TransformMode::Millimeter2D:
		if (requireCoord)
		{
			objectified["x"] = int(Object.location.translation()[0]*1000.0+1500.0);
			objectified["y"] = int(Object.location.translation()[1]*1000.0+1000.0);
		}

		objectified["r"] = int(rotZdeg*10)/10.0;
		
		break;

	case TransformMode::Float3D:
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				objectified[string("r") + to_string(i)][string("c") + to_string(j)] = Object.location.matrix(i,j);
			}
		}
		break;

	default:
		break;
	}
	if (Object.type == ObjectType::SolarPanel)
	{
		bool teamyellow = false, teamblue = false;
		if (rotZdeg > 5)
		{
			teamyellow = true;
		}
		if (rotZdeg < -5)
		{
			teamblue = true;
		}
		if (rotZdeg > 155)
		{
			teamblue = true;
		}
		if (rotZdeg < -155)
		{
			teamyellow = true;
		}
		static const array<string, 4> teams = {"NONE", "YELLOW", "BLUE", "BLUE_YELLOW"};
		objectified["team"] = teams[teamblue*2+teamyellow]; 
	}
	return objectified;
}

bool JsonListener::GetData(const json &filter, json &Response)
{
	if (!Parent)
	{
		return false;
	}
	if (!Parent->ExternalRunner)
	{
		return false;
	}
	vector<CameraFeatureData> FeatureData = Parent->ExternalRunner->GetFeatureData(); 
	vector<ObjectData> ObjData = Parent->ExternalRunner->GetObjectData();
	set<ObjectType> AllowedTypes;
	set<string> filterStrings;
	for (auto &elem : filter)
	{
		filterStrings.emplace(elem);
	}
	bool hasall = filterStrings.find("ALL") != filterStrings.end();

	auto has_filter = [&filterStrings, hasall](string match){return filterStrings.find(match) != filterStrings.end() || hasall;};
	
	json jsondataarray = json::array({});
	for (auto& [type,name] : ObjectTypeNames)
	{
		string java_name = JavaCapitalize(name);
		if (has_filter(java_name))
		{
			AllowedTypes.insert(type);
		}
	}
	bool Has3DData = false;
	for (auto &Object : ObjData)
	{
		if (AllowedTypes.find(Object.type) == AllowedTypes.end())
		{
			continue;
		}
		json objectified = ObjectToJson(Object);
		jsondataarray.push_back(objectified);
		Has3DData = true;
	}
	if (Has3DData)
	{
		Response["data3D"] = jsondataarray;
	}
	
	bool Has2DData = false;
	json jsonfeaturearray = json::array();
	for (auto& data : FeatureData)
	{
		bool CameraDetected = false;
		json cameradata;
		cameradata["name"] = JavaCapitalize(data.CameraName);
		cameradata["width"] = data.FrameSize.width;
		cameradata["height"] = data.FrameSize.height;
		cv::Size2d fov = GetCameraFOV(data.FrameSize, data.CameraMatrix);
		cameradata["xfov"] = fov.width;
		cameradata["yfov"] = fov.height;
		if (has_filter("ARUCO"))
		{
			cameradata["arucoObjects"] = json::array();
			for (size_t i = 0; i < data.ArucoIndices.size(); i++)
			{
				json aruco;
				aruco["index"] = data.ArucoIndices[i];
				aruco["corners"] = json::array();
				for (size_t j = 0; j < data.ArucoCorners[i].size(); j++)
				{
					aruco["corners"][j]["x"] = int(data.ArucoCorners[i][j].x);
					aruco["corners"][j]["y"] = int(data.ArucoCorners[i][j].y);
				}
				cameradata["arucoObjects"].push_back(aruco);
				CameraDetected = true;
			}
		}
		if (has_filter("YOLO"))
		{
			cameradata["yoloObjects"] = json::array();
			for (size_t i = 0; i < data.YoloDetections.size(); i++)
			{
				json yolodet;
				auto& det = data.YoloDetections[i];
				yolodet["index"] = det.Class;
				auto &Corner = det.Corners;
				yolodet["tlx"] = int(Corner.tl().x);
				yolodet["tly"] = int(Corner.tl().y);
				yolodet["brx"] = int(Corner.br().x);
				yolodet["bry"] = int(Corner.br().y);
				yolodet["confidence"] = int(det.Confidence*100);
				cameradata["yoloObjects"].push_back(yolodet);
				CameraDetected = true;
			}
		}
		if (CameraDetected)
		{
			jsonfeaturearray.push_back(cameradata);
			Has2DData = true;
		}
	}
	if (Has2DData)
	{
		Response["data2D"] = jsonfeaturearray;
	}
	
	return true;
}

bool JsonListener::GetImage(double reduction, json &Response)
{
	if (!Parent)
	{
		return false;
	}
	if (!Parent->ExternalRunner)
	{
		return false;
	}
	auto cameras = Parent->ExternalRunner->GetImage();
	Response["cameras"] = json::array({});
	for (size_t i = 0; i < cameras.size(); i++)
	{
		cv::UMat image;
		auto & this_cam = cameras[i];
		if (reduction <= 1)
		{
			image = this_cam.Image;
		}
		else
		{
			cv::resize(this_cam.Image, image, cv::Size(0,0), 1/reduction, 1/reduction);
		}
		std::vector<uchar> jpgenc, b64enc;
		cv::imencode(".jpg", image, jpgenc);
		size_t b64size = jpgenc.size()*4/3+16;
		b64enc.resize(b64size);
		base64_encode(reinterpret_cast<char*>(jpgenc.data()), jpgenc.size(), 
			reinterpret_cast<char*>(b64enc.data()), &b64size, 0);
		b64enc.resize(b64size);
		json this_camera_json;
		this_camera_json["name"] = this_cam.CameraName;
		this_camera_json["data"] = b64enc;
		Response["cameras"].push_back(this_camera_json);
	}
	return true;
}

void JsonListener::ReceiveImage(const json &Query)
{
	if (!Query.contains("data"))
	{
		cout << "Received homework without any image !" << endl << Query << endl;
		return;
	}
	std::vector<uchar> jpgenc;
	string b64enc = Query.at("data");
	size_t jpegsize = b64enc.size()*3/4+16;
	jpgenc.resize(jpegsize);
	if(int decerr = base64_decode(b64enc.c_str(), b64enc.size(), reinterpret_cast<char*>(jpgenc.data()), &jpegsize, 0))
	{
		cerr << "Failed to decode image: " << decerr << endl;
		return;
	}
	cv::UMat decoded;
	cv::imdecode(jpgenc, 0).copyTo(decoded);
	if (!Parent)
	{
		return;
	}
	if (!Parent->InternalRunner)
	{
		return;
	}
	//do stuff here
	//Parent->InternalRunner->Inject()
}

void JsonListener::HandleQuery(const json &Query)
{
	const string &ActionStr = Query.value("action", "invalid");
	json Response;
	Response["action"] = ActionStr;
	if (Query.contains("index"))
	{
		Response["index"] = Query["index"];
	}
	if (ActionStr == "ALIVE")
	{
		Response["status"] = "OK";
	}
	else if (ActionStr == "CONFIG") //idk, do something ?
	{
		if (Query.contains("mode"))
		{
			string mode = Query.value("mode", "none");
			Response["status"] = "OK";
			if (mode == "MILLIMETER_2D")
			{
				ObjectMode = TransformMode::Millimeter2D;
			}
			else if (mode == "FLOAT_2D")
			{
				ObjectMode = TransformMode::Float2D;
			}
			else if (mode == "FLOAT_3D")
			{
				ObjectMode = TransformMode::Float3D;
			}
			else
			{
				Response["status"] = "UNKNOWN_MODE";
			}
		}
	}
	else if (ActionStr == "STATUS") //How are the cameras doing ?
	{
		Response["status"] = "UNIMPLEMENTED";
	}
	else if (ActionStr == "DATA") //2D or 3D data
	{
		if (Query.contains("filter") && Query.at("filter").is_array())
		{
			GetData(Query["filter"], Response);
			Response["status"] = "OK";
		}
		else
		{
			Response["status"] = "MISSING_FILTER";
			Response["log"] = "wrong or missing filter, must be an array";
		}
		
	}
	else if (ActionStr == "IMAGE")
	{
		double reduction = Query.value("Reduction", 1.0);
		GetImage(reduction, Response);
		Response["status"] = "OK";
	}
	else if (ActionStr == "PROCESS") //oh no, homework !
	{
		Response["status"] = "UNIMPLEMENTED";
	}
	else if (ActionStr == "EXIT")
	{
		killed = true;
		Response["status"] = "OK";
		Response["log"] = "Committing seppuku !";
	}
	else
	{
		Response["status"] = "KO";
		Response["log"] = "you ok there bud ?";
	}
	
	SendJson(Response);
}

void JsonListener::HandleResponse(const json &Response)
{
	(void) Response;
}

bool JsonListener::IsQuery(const json &object)
{
	return object.contains("action") && !object.contains("status");
}

void JsonListener::HandleJson(const string &command)
{
	json parsed;
	try
	{
		parsed = json::parse(command);
	}
	catch(const json::exception& e)
	{
		std::cerr << e.what() << '\n';
		return;
	}
	
	
	if (parsed.is_discarded())
	{
		return;
	}
	LastAliveReceived = chrono::steady_clock::now();
	if (IsQuery(parsed))
	{
		cout << "Received action : " << command << endl;
		HandleQuery(parsed);
	}
	else
	{
		HandleResponse(parsed);
	}
}

void JsonListener::SendJson(const json &object)
{
	string SendBuffer = object.dump() + "\n";

	if(!Transport->Send(SendBuffer.data(), SendBuffer.length(), ClientName))
	{
		killed = true;
	}
}

void JsonListener::CheckAlive()
{
	auto settings = GetKeepAliveSettings();
	if (settings.kick_delay <= 0)
	{
		return;
	}
	
	chrono::duration<double> TimeSinceLastAliveReceived = chrono::steady_clock::now() - LastAliveReceived;
	chrono::duration<double> TimeSinceLastAliveSent = chrono::steady_clock::now() - LastAliveSent;
	if (TimeSinceLastAliveReceived.count() > settings.kick_delay)
	{
		cout << "Got no activity from " << ClientName << ", closing..." << endl;
		killed = true;
		return;
	}
	
	if (settings.poke_delay >= 0 && TimeSinceLastAliveReceived.count() > settings.poke_delay && TimeSinceLastAliveSent.count() > settings.poke_delay)
	{
		//cout << "Asking " << ClientName << " if it's alive..." << endl;
		LastAliveSent = chrono::steady_clock::now();
		json query;
		query["action"] = "ALIVE";
		query["index"] = sendIndex++;
		SendJson(query);
	}
}

void JsonListener::ThreadEntryPoint()
{
	while (!killed)
	{
		CheckAlive();
		
		char bufferraw[1<<10];
		int numreceived = Transport->Receive(bufferraw, sizeof(bufferraw), ClientName, false);
		if (numreceived <= 0)
		{
			this_thread::sleep_for(chrono::microseconds(500));
			continue;
		}
		int rcvbufstartpos = 0;
		int rcvbufinsertpos = ReceiveBuffer.size();

		ReceiveBuffer.insert(ReceiveBuffer.end(), bufferraw, bufferraw+numreceived);
		
		for (size_t i = rcvbufinsertpos; i < ReceiveBuffer.size(); i++)
		{
			if (ReceiveBuffer[i] != '\n')
			{
				continue;
			}
			int commandlen = i-rcvbufstartpos-1;
			if (commandlen<2)
			{
				
			}
			else
			{
				string command(ReceiveBuffer.begin() + rcvbufstartpos, ReceiveBuffer.begin() + i);
				HandleJson(command);
			}
			rcvbufstartpos=i+1;
		}
		rcvbufstartpos = min<int>(ReceiveBuffer.size(), rcvbufstartpos);
		if (rcvbufstartpos != 0)
		{
			ReceiveBuffer.erase(ReceiveBuffer.begin(), ReceiveBuffer.begin()+rcvbufstartpos);
		}
	}
}