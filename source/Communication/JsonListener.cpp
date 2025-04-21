#include "Communication/JsonListener.hpp"

#include <Transport/ConnectionToken.hpp>
#include <Communication/TCPJsonHost.hpp>
#include <Cameras/ImageTypes.hpp>
#include <EntryPoints/CDFRCommon.hpp>
#include <EntryPoints/CDFRExternal.hpp>
#include <EntryPoints/CDFRInternal.hpp>
#include <Misc/math3d.hpp>
#include <Misc/math2d.hpp>
#include <Misc/GlobalConf.hpp>
#include <Transport/thread-rename.hpp>

#include <opencv2/imgcodecs.hpp>
#include <libbase64.h>

#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <set>

using namespace std;
using namespace nlohmann;

JsonListener::JsonListener(shared_ptr<ConnectionToken> InToken, TCPJsonHost* InParent)
	:token(InToken), Parent(InParent)
{
	LastAliveSent = chrono::steady_clock::now();
	LastAliveReceived = LastAliveSent;
	Start();
	//cout << "Json listen thread for " << token->GetConnectionName() << " started" << endl;
}

JsonListener::~JsonListener()
{
	//cout << "Json listen thread for " << token->GetConnectionName() << " stoppped" << endl;
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
		if (value == i.second.JavaName)
		{
			Team = i.first;
			break;
		}
	}
	return Team;
}

optional<json> JsonListener::ObjectToJson(const ObjectData& Object)
{
	json objectified;
	const auto &ObjectTypeConfig = ObjectTypeNames.at(Object.type);
	if (!ObjectTypeConfig.Sendable)
	{
		return nullopt;
	}
	
	objectified["type"] = ObjectTypeConfig.JavaName;
	objectified["name"] = JavaCapitalize(Object.name);
	if (Object.metadata.size() > 0)
	{
		objectified["metadata"] = Object.metadata;
	}
	objectified["age"] = chrono::duration_cast<chrono::milliseconds>(ObjectData::Clock::now() - Object.LastSeen).count();
	
	bool requireCoord = ObjectTypeConfig.WantPosition;
	bool requireRot = ObjectTypeConfig.WantRotation;
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
		if (requireRot)
		{
			objectified["r"] = rotZ;
		}
		
		break;
	case TransformMode::Millimeter2D:
		if (requireCoord)
		{
			objectified["x"] = (int)round(Object.location.translation()[0]*1000.0+1500.0);
			objectified["y"] = (int)round(Object.location.translation()[1]*1000.0+1000.0);
		}
		if (requireRot)
		{
			objectified["r"] = (int)round(rotZdeg);
		}
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
	return objectified;
}

std::set<ObjectType> JsonListener::GetFilterClasses(const nlohmann::json &filter)
{
	set<string> filterStrings;
	if (filter.is_array())
	{
		for (auto &elem : filter)
		{
			filterStrings.emplace(elem);
		}
	}
	else if (filter.is_string())
	{
		filterStrings.emplace(filter);
	}
	else
	{
		return {};
	}
	set<ObjectType> AllowedTypes;
	for (auto& [type,name] : ObjectTypeNames)
	{
		const string &java_name = name.JavaName;
		if (filterStrings.find(java_name) != filterStrings.end())
		{
			AllowedTypes.insert(type);
		}
	}
	if (AllowedTypes.find(ObjectType::All) != AllowedTypes.end())
	{
		AllowedTypes.insert(ObjectType::Data2D);
		AllowedTypes.insert(ObjectType::Data3D);
	}
	return AllowedTypes;
}

ObjectData::TimePoint JsonListener::GetCutoffTime(const nlohmann::json &Query)
{
	int maxagems = Query.at("data").value("maxAge", 0);
	ObjectData::TimePoint OldCutoff;
	if (maxagems >0)
	{
		OldCutoff = ObjectData::Clock::now() - chrono::milliseconds(maxagems);
	}
	return OldCutoff;
}

bool JsonListener::GetData(const json &Query, json &Response)
{
	if (!Query.contains("data"))
	{
		return false;
	}
	auto &QueryData = Query.at("data");
	if (!QueryData.contains("filters"))
	{
		return false;
	}
	if (!QueryData.at("filters").is_array())
	{
		return false;
	}
	ObjectData::TimePoint OldCutoff = GetCutoffTime(Query);
	 
	vector<CameraFeatureData> FeatureData = Parent->ExternalRunner->GetFeatureData();
	vector<ObjectData> ObjData = Parent->ExternalRunner->GetObjectData();
	set<ObjectType> AllowedTypes = GetFilterClasses(QueryData.at("filters"));

	json jsondataarray = json::array({});
	bool has3D = AllowedTypes.find(ObjectType::Data3D) != AllowedTypes.end(); 
	bool has2D = AllowedTypes.find(ObjectType::Data2D) != AllowedTypes.end(); 
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
		auto objectified = ObjectToJson(Object);
		if (objectified.has_value())
		{
			jsondataarray.push_back(objectified.value());
			Has3DData = true;
		}
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
		cameradata["xfov"] = round(fov.width * 180 / M_PI * 100) / 100;
		cameradata["yfov"] = round(fov.height * 180 / M_PI * 100) / 100;
		if (has2D || AllowedTypes.find(ObjectType::Aruco) != AllowedTypes.end())
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
		if (has2D || AllowedTypes.find(ObjectType::Yolo) != AllowedTypes.end())
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

	Response["status"] = "OK";
	return true;
}

bool JsonListener::GetZone(const json &Query, json &Response)
{
	if (!Query.contains("data"))
	{
		return false;
	}
	auto &QueryData = Query.at("data");
	set<ObjectType> AllowedTypes;
	if (!QueryData.contains("zones"))
	{
		return false;
	}
	if (!QueryData.at("zones").is_array())
	{
		return false;
	}
	if (!QueryData.contains("classes"))
	{
		AllowedTypes.insert(ObjectType::Fragile);
		AllowedTypes.insert(ObjectType::Resistant);
		AllowedTypes.insert(ObjectType::Pot);
		AllowedTypes.insert(ObjectType::PottedPlant);
	}
	else if (!QueryData.at("classes").is_array())
	{
		return false;
	}
	else
	{
		AllowedTypes = GetFilterClasses(QueryData.at("classes"));
	}
	
	auto ObjData = Parent->ExternalRunner->GetObjectData();

	vector<pair<string, cv::Rect2d>> PositionFilters;
	for (auto &elem : QueryData.at("zones"))
	{
		try
		{
			double cx = elem.at("cx");
			double cy = elem.at("cy");
			double dx = elem.at("dx");
			double dy = elem.at("dy");
			PositionFilters.emplace_back(elem.at("name"), cv::Rect2d(cx-dx/2.0, cy-dy/2.0, dx, dy));
		}
		catch(const json::exception& e)
		{
		}
	}

	if (ObjectMode == TransformMode::Millimeter2D)
	{
		cv::Point2d offset(1500,1000);
		for (auto &zone : PositionFilters)
		{
			cv::Point2d tl = zone.second.tl();
			cv::Size2d size = zone.second.size();
			tl = (tl-offset)/1000;
			size /= 1000.0;
			zone.second = cv::Rect2d(tl, size);
		}
	}

	auto OldCutoff = GetCutoffTime(Query);
	if (OldCutoff == ObjectData::TimePoint())
	{
		OldCutoff = ObjectData::Clock::now() - chrono::seconds(3);
	}
	
	set<string> SeenZones;

	for (auto &Object : ObjData)
	{
		if (AllowedTypes.find(Object.type) == AllowedTypes.end())
		{
			continue;
		}
		if (Object.LastSeen < OldCutoff)
		{
			continue;
		}
		for (auto &zone : PositionFilters)
		{
			cv::Vec3d pos3d = Object.location.translation();
			cv::Vec2d pos2d(pos3d.val);
			if (zone.second.contains<double>(pos2d))
			{
				SeenZones.insert(zone.first);
				break;
			}
		}
	}
	Response["data"]["zones"] = SeenZones;
	Response["status"] = "OK";
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
		team = GetOtherTeam(SavedTeam);
		if (team == CDFRTeam::Unknown)
		{
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
		if (i.name.rfind(TeamNames.at(team).JavaName, 0) != 0) //must start with robot
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
	if (ActionStr == "START")
	{
		CDFRCommon::ExternalSettings.record = true;
		CDFRCommon::ExternalSettings.RecordInterval = 0;
		CDFRCommon::InternalSettings.record = true;
		CDFRCommon::InternalSettings.RecordInterval = 0;
		if (Parent)
		{
			if (Parent->ExternalRunner)
			{
				Parent->ExternalRunner->SetCameraLock(true);
				CDFRTeam team = Parent->ExternalRunner->GetTeam();
				if (team != CDFRTeam::Unknown)
				{
					Parent->ExternalRunner->SetTeamLock(team);
				}
			}
		}
		Response["status"] = "OK";
		goto send;
	}
	if (ActionStr == "END")
	{
		CDFRCommon::ExternalSettings.record = false;
		CDFRCommon::InternalSettings.record = false;
		if (Parent)
		{
			if (Parent->ExternalRunner)
			{
				Parent->ExternalRunner->SetCameraLock(false);
				Parent->ExternalRunner->SetTeamLock(CDFRTeam::Unknown);
			}
		}
		Response["status"] = "OK";
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
				Response["data"]["team"] = TeamNames.at(Parent->ExternalRunner->GetTeam()).JavaName;
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
			if(GetData(Query, Response))
			{
			}
			else
			{
				Response["status"] = "ERROR";
			}
			goto send;
		}
		if (ActionStr == "ZONE") //Get if zone empty or not
		{
			if(GetZone(Query, Response))
			{
			}
			else
			{
				Response["status"] = "ERROR";
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

	if (!token->IsConnected())
	{
		killed = true;
		return;
	}
	if (!token->Send(SendBuffer.data(), SendBuffer.size()))
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
		cout << "Got no activity from " << token->GetConnectionName() << ", closing..." << endl;
		killed = true;
		return;
	}

	if (settings.poke_delay > 0 && TimeSinceLastAliveReceived.count() > settings.poke_delay && TimeSinceLastAliveSent.count() > settings.poke_delay)
	{
		LastAliveSent = chrono::steady_clock::now();
		if (!token->IsConnected())
		{
			killed = true;
			return;
		}
		if (!token->Send(" ", 1))
		{
			cout << "Client " << token->GetConnectionName() << " disconnect while checking alive, closing..." << endl;
			killed = true;
		}
	}
}

void JsonListener::ThreadEntryPoint()
{
	if (!token)
	{
		killed = true;
		return;
	}
	auto transport = token->GetParent();
	if (!transport)
	{
		killed = true;
		return;
	}
	string ThreadName = string("Json ") + token->GetConnectionName();
	SetThreadName(ThreadName.c_str());
	while (!killed && token->IsConnected())
	{
		CheckAlive();

		array<char, 1<<10> bufferraw;
		auto numreceived = token->Receive(bufferraw.data(), bufferraw.size());
		if (!numreceived.has_value())
		{
			killed = true;
			break;
		} else if (numreceived.value() == 0) {
			this_thread::sleep_for(chrono::microseconds(500));
		}
		int rcvbufstartpos = 0;
		int rcvbufinsertpos = ReceiveBuffer.size();

		ReceiveBuffer.insert(ReceiveBuffer.end(), bufferraw.data(), bufferraw.data() + numreceived.value());

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
	token->Disconnect();
	killed = true;
}