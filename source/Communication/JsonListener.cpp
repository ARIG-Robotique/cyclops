#include "Communication/JsonListener.hpp"
#include <Communication/Transport/TCPTransport.hpp>
#include <Communication/TCPJsonHost.hpp>
#include <Cameras/ImageTypes.hpp>
#include <EntryPoints/CDFRExternal.hpp>
#include <EntryPoints/CDFRInternal.hpp>
#include <Misc/math3d.hpp>
#include <Misc/math2d.hpp>
#include <Misc/GlobalConf.hpp>

#include <opencv2/imgcodecs.hpp>
#include <libbase64.h>

#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <set>

using namespace std;
using namespace nlohmann;

JsonListener::JsonListener(TCPTransport* InTransport, string InClientName, TCPJsonHost* InParent)
	:Transport(InTransport), ClientName(InClientName), Parent(InParent)
{
	LastAliveSent = chrono::steady_clock::now();
	LastAliveReceived = LastAliveSent;
	Start();
	cout << "Json listen thread for " << ClientName << " started" << endl;
}

JsonListener::~JsonListener()
{
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

CDFRTeam JsonListener::StringToTeam(string team)
{
	string value = JavaCapitalize(team);
	CDFRTeam Team = CDFRTeam::Unknown;
	for (auto &i : TeamNames)
	{
		if (value == JavaCapitalize(i.second))
		{
			Team = i.first;
			break;
		}
	}
	return Team;
}

json JsonListener::ObjectToJson(const ObjectData& Object)
{
	json objectified;
	objectified["type"] = JavaCapitalize(ObjectTypeNames.at(Object.type));
	objectified["name"] = JavaCapitalize(Object.name);
	if (Object.metadata.size() > 0)
	{
		objectified["metadata"] = Object.metadata;
	}
	objectified["age"] = chrono::duration_cast<chrono::milliseconds>(ObjectData::Clock::now() - Object.LastSeen).count();
	
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
		static const array<string, 4> teams = {"AUCUNE", "JAUNE", "BLEU", "JAUNE_ET_BLEU"};
		objectified["team"] = teams[teamblue*2+teamyellow];
	}
	return objectified;
}

bool JsonListener::GetData(const json &Query, json &Response)
{
	if (!Parent)
	{
		return false;
	}
	if (!Parent->ExternalRunner)
	{
		return false;
	}
	auto &filter = Query["data"]["filters"];
	int maxagems = Query["data"].value("maxAge", 0);
	ObjectData::TimePoint OldCutoff;
	if (maxagems >0)
	{
		OldCutoff = ObjectData::Clock::now() - chrono::milliseconds(maxagems);
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
	bool has3D = filterStrings.find("DATA3D") != filterStrings.end() || hasall;
	bool has2D = filterStrings.find("DATA2D") != filterStrings.end() || hasall;

	auto has_filter = [&filterStrings, hasall](string match){return filterStrings.find(match) != filterStrings.end();};

	json jsondataarray = json::array({});
	for (auto& [type,name] : ObjectTypeNames)
	{
		string java_name = JavaCapitalize(name);
		if (has_filter(java_name) && java_name != "CAMERA")
		{
			AllowedTypes.insert(type);
		}
	}
	bool Has3DData = false;
	for (auto &Object : ObjData)
	{
		if (!has3D && AllowedTypes.find(Object.type) == AllowedTypes.end())
		{
			continue;
		}
		if (Object.LastSeen < OldCutoff)
		{
			continue;
		}
		json objectified = ObjectToJson(Object);
		jsondataarray.push_back(objectified);
		Has3DData = true;
	}
	if (Has3DData)
	{
		Response["data"]["data3D"] = jsondataarray;
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
		if (has_filter("ARUCO") || has2D)
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
		if (has_filter("YOLO") || has2D)
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
		Response["data"]["data2D"] = jsonfeaturearray;
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
	Response["data"]["cameras"] = json::array({});
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
		this_camera_json["data"] = reinterpret_cast<char*>(b64enc.data());
		Response["data"]["cameras"].push_back(this_camera_json);
	}
	return true;
}

bool JsonListener::GetStartingZone(const nlohmann::json query, nlohmann::json &response)
{
	CDFRTeam team = StringToTeam(query.value("team", ""));
	if (team == CDFRTeam::Unknown)
	{
		CDFRTeam SavedTeam = Parent->ExternalRunner->GetTeam();
		switch (SavedTeam)
		{
		case CDFRTeam::Yellow :
			team = CDFRTeam::Blue;
			break;
		case CDFRTeam::Blue : 
			team = CDFRTeam::Yellow;
			break;
		default:
			response["status"] = "ERROR";
			response["errorMessage"] = "No team selected";
			return false;
		}
		
		
	}
	auto data = Parent->ExternalRunner->GetObjectData();
	ObjectData robot;
	robot.LastSeen = ObjectData::TimePoint();
	for (auto &&i : data)
	{
		if (i.type != ObjectType::Robot)
		{
			continue;
		}
		if (i.name.rfind(TeamNames.at(team), 0) != 0) //must start with robot
		{
			continue;
		}
		if (i.LastSeen < robot.LastSeen)
		{
			continue;
		}
		
		robot = i;
	}
	if (robot.LastSeen == ObjectData::TimePoint())
	{
		response["status"] = "NO_DATA";
		return true;
	}
	
	string position = "";
	position += robot.location.translation()[0] > 0 ? "EAST" : "WEST";
	if (robot.location.translation()[1]>0.38)
	{
		position += "_NORTH";
	}
	else if (robot.location.translation()[1]<-0.38)
	{
		position += "_SOUTH";
	}
	
	response["status"] = "OK";
	response["zone"] = position;
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
		goto send;
	}
	if (ActionStr == "CONFIG") //idk, do something ?
	{
		if (Query.contains("data") && Query["data"].contains("mode"))
		{
			string mode = Query["data"].value("mode", "none");
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
				Response["status"] = "ERROR";
				Response["errorMessage"] = "Unknown Mode";
			}
		}
		goto send;
	}
	if (ActionStr == "EXIT")
	{
		killed = true;
		Response["status"] = "OK";
		goto send;
	}
	if (ActionStr == "STATUS")
	{
		Response["status"] = "OK";
		Response["data"]["parent"] = Parent ? "OK" : "ERROR";
		Response["data"]["team"] = "INCONNUE";
		Response["data"]["idle"] = false;
		ostringstream statusbuilder;
		if (!Parent)
		{
			statusbuilder << "Json Listener missing parent; ";
			Response["data"]["externalRunner"] = "ERROR";
			Response["data"]["internalRunner"] = "ERROR";
		}
		else
		{
			Response["data"]["externalRunner"] = Parent->ExternalRunner ? "OK" : "ERROR";
			Response["data"]["internalRunner"] = Parent->InternalRunner ? "OK" : "ERROR";
			if (!Parent->ExternalRunner)
			{
				statusbuilder << "Json Listener missing external runner; ";
			}
			else
			{
				statusbuilder << "External runner doing fine; ";
				Response["data"]["team"] = JavaCapitalize(TeamNames.at(Parent->ExternalRunner->GetTeam()));
				Response["data"]["idle"] = Parent->ExternalRunner->GetIdle();
			}

			if (!Parent->InternalRunner)
			{
				statusbuilder << "Json Listener missing internal runner; ";
			}
			else
			{
				statusbuilder << "Internal runner doing fine; ";
			}
		}
		Response["data"]["statusMessage"] = statusbuilder.str();
		try
		{
			int charge_full_value, charge_now_value;
			ifstream charge_full_file("/sys/class/power_supply/BAT1/charge_full"), charge_now_file("/sys/class/power_supply/BAT1/charge_now");
			charge_full_file >> charge_full_value;
			charge_now_file >> charge_now_value;
			Response["data"]["battery"] = 100*charge_now_value/charge_full_value;
		}
		catch(const std::exception& e)
		{
			std::cerr << "Failed to get battery charge: " << e.what() << '\n';
		}
		

		Response["data"]["mode"] = TransformModeNames.at(ObjectMode);

		goto send;
	}
	if (Parent && Parent->InternalRunner)
	{
		if (ActionStr == "PROCESS") //oh no, homework !
		{
			Response["status"] = "ERROR";
			Response["errorMessage"] = "Not implemented";
			goto send;
		}
	}

	if (Parent && Parent->ExternalRunner)
	{
		if (Parent->ExternalRunner->IsKilled())
		{
			return;
		}
		
		if (ActionStr == "DATA") //2D or 3D data
		{
			if (Query.contains("data") && Query["data"].contains("filters") && Query["data"].at("filters").is_array())
			{
				GetData(Query, Response);
				Response["status"] = "OK";
			}
			else
			{
				Response["status"] = "ERROR";
				Response["errorMessage"] = "Wrong or missing filters, must be an array";
			}
			goto send;
		}
		if (ActionStr == "IMAGE")
		{
			double reduction = 1.0;
			if (Query.contains("data")) {
				reduction = Query["data"].value("reduction", 1.0);
			}
			GetImage(reduction, Response);
			Response["status"] = "OK";
			goto send;
		}
		if (ActionStr == "IDLE")
		{
			bool value = false;
			if (Query.contains("data")) {
				value = Query["data"].value("value", false);
			}
			Parent->ExternalRunner->SetIdle(value);
			Response["status"] = "OK";
			goto send;
		}
		if (ActionStr == "TEAM")
		{
			if (!Query.contains("data") || !Query["data"].contains("team")) {
				Response["status"] = "ERROR";
				Response["errorMessage"] = "Error: data.value: Team must be specified";
			} else {
				Parent->ExternalRunner->SetTeamLock(StringToTeam(Query["data"]["team"]));
				Response["status"] = "OK";
			}
			goto send;
		}
		if (ActionStr == "LOCK_CAMERA")
		{
			Parent->ExternalRunner->SetCameraLock(Query.value("data.value", false));
			Response["status"] = "OK";
			goto send;
		}
		if (ActionStr == "STARTING_ZONE")
		{
			GetStartingZone(Query, Response);
			goto send;
		}

	}

	{
		Response["status"] = "ERROR";
		Response["log"] = "Action not recognized";
		goto send;
	}
send:
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

	chrono::duration<double> TimeSinceLastAliveReceived = chrono::steady_clock::now() - LastAliveReceived;
	chrono::duration<double> TimeSinceLastAliveSent = chrono::steady_clock::now() - LastAliveSent;
	if (settings.kick_delay > 0 && TimeSinceLastAliveReceived.count() > settings.kick_delay)
	{
		cout << "Got no activity from " << ClientName << ", closing..." << endl;
		killed = true;
		return;
	}

	if (settings.poke_delay > 0 && TimeSinceLastAliveReceived.count() > settings.poke_delay && TimeSinceLastAliveSent.count() > settings.poke_delay)
	{
		LastAliveSent = chrono::steady_clock::now();
		if (!Transport->Send(" ", 1, ClientName))
		{
			cout << "Client " << ClientName << " disconnect while checking alive, closing..." << endl;
			killed = true;
		}

	}
}

void JsonListener::ThreadEntryPoint()
{
	while (!killed)
	{
		CheckAlive();

		array<char, 1<<10> bufferraw;
		int numreceived = Transport->Receive(bufferraw.data(), bufferraw.size(), ClientName, false);
		if (numreceived < 0)
		{
			this_thread::sleep_for(chrono::microseconds(500));
			continue;
		} else if (numreceived == 0) {
			killed = true;
			break;
		}
		int rcvbufstartpos = 0;
		int rcvbufinsertpos = ReceiveBuffer.size();

		ReceiveBuffer.insert(ReceiveBuffer.end(), bufferraw.data(), bufferraw.data() + numreceived);

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