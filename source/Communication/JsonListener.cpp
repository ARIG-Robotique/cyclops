#include "Communication/JsonListener.hpp"
#include <Communication/Transport/TCPTransport.hpp>
#include <EntryPoints/CDFRExternal.hpp>
#include <Communication/TCPJsonHost.hpp>
#include <Misc/math3d.hpp>
#include <Misc/math2d.hpp>

#include <nlohmann/json.hpp>
#include <iostream>

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
	ListenThread = new thread(&JsonListener::ThreadEntryPoint, this);
	cout << "Json listen thread for " << ClientName << " started" << endl;
}

JsonListener::~JsonListener()
{
	cout << "Json listen thread for " << ClientName << " stoppped" << endl;
	killed = true;
	if (ListenThread)
	{
		ListenThread->join();
		delete ListenThread;
	}
}

json JsonListener::ObjectToJson(const ObjectData& Object)
{
	json objectified;
	objectified["type"] = ObjectTypeNames.at(Object.type);
	switch (ObjectMode)
	{
	case TransformMode::Float2D:
		objectified["x"] = Object.location.translation()[0];
		objectified["y"] = Object.location.translation()[1];
		objectified["r"] = GetRotZ(Object.location.rotation());
		break;
	case TransformMode::Millimeter2D:
		objectified["x"] = Object.location.translation()[0]*1000.0+1500.0;
		objectified["y"] = Object.location.translation()[1]*1000.0+1000.0;
		objectified["r"] = -GetRotZ(Object.location.rotation())*180.0/M_PI;
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

json JsonListener::GetData(json filter)
{
	if (!Parent)
	{
		return "[]";
	}
	if (!Parent->ExternalRunner)
	{
		return "[]";
	}
	vector<CameraFeatureData> FeatureData; vector<ObjectData> ObjData;
	Parent->ExternalRunner->GetData(FeatureData, ObjData);
	set<ObjectType> AllowedTypes;
	bool hasall = filter.contains("all");
	json JsonResponse;
	json jsondataarray = json::array({});
	for (auto& [type,name] : ObjectTypeNames)
	{
		if (filter.contains(name) || hasall)
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
		JsonResponse["3D Data"] = jsondataarray;
	}
	
	bool Has2DData = false;
	json jsonfeaturearray = json::array();
	for (auto& data : FeatureData)
	{
		bool CameraDetected = false;
		json cameradata;
		cameradata["name"] = data.CameraName;
		cameradata["width"] = data.FrameSize.width;
		cameradata["height"] = data.FrameSize.height;
		cv::Size2d fov = GetCameraFOV(data.FrameSize, data.CameraMatrix);
		cameradata["xfov"] = fov.width;
		cameradata["yfov"] = fov.height;
		if (filter.contains("Aruco") || hasall)
		{
			cameradata["arucos"] = json::array();
			for (int i = 0; i < data.ArucoIndices.size(); i++)
			{
				json aruco;
				aruco["index"] = data.ArucoIndices[i];
				aruco["corners"] = json::array();
				for (int j = 0; j < data.ArucoCorners[i].size(); j++)
				{
					aruco["corners"][j]["x"] = data.ArucoCorners[i][j].x;
					aruco["corners"][j]["y"] = data.ArucoCorners[i][j].y;
				}
				cameradata["arucos"].push_back(aruco);
				CameraDetected = true;
			}
		}
		if (filter.contains("Yolo") || hasall)
		{
			cameradata["yolo"] = json::array();
			for (int i = 0; i < data.YoloIndices.size(); i++)
			{
				json yolodet;
				yolodet["index"] = data.YoloIndices[i];
				auto &Corner = data.YoloCorners[i];
				yolodet["tlx"] = Corner.tl().x;
				yolodet["tly"] = Corner.tl().y;
				yolodet["brx"] = Corner.br().x;
				yolodet["bry"] = Corner.br().y;
				cameradata["yolo"].push_back(yolodet);
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
		JsonResponse["2D Data"] = jsonfeaturearray;
	}
	
	return JsonResponse;
}

void JsonListener::HandleQuery(const json &Query)
{
	const string &Command = Query.value("query", "invalid");
	int index = Query.value("index", -1);
	json Response;
	Response["index"] = index;

	if (Command == "alive")
	{
		Response["response"] = true;
	}
	else if (Command == "config") //idk, do something ?
	{
		if (Query.contains("mode"))
		{
			string mode = Query.value("mode", "none");
			Response["response"]["mode"] = "mode accepted";
			if (mode == "Millimeter2D")
			{
				ObjectMode = TransformMode::Millimeter2D;
			}
			else if (mode == "Float2D")
			{
				ObjectMode = TransformMode::Float2D;
			}
			else if (mode == "Float3D")
			{
				ObjectMode = TransformMode::Float3D;
			}
			else
			{
				Response["response"]["mode"] = "unknown mode";
			}
			
		}
	}
	else if (Command == "status") //How are the cameras doing ?
	{
		
	}
	else if (Command == "data") //2D or 3D data
	{
		if (Query.contains("filter") && Query.at("filter").is_array())
		{
			Response += GetData(Query["filter"]);
		}
		else
		{
			Response["response"] = "wrong or missing filter, must be an array";
		}
		
	}
	else if (Command == "processing") //oh no, homework !
	{

	}
	else if (Command == "seppuku" || Command == "kill")
	{
		killed = true;
		Response["response"] = "aight, imma commit seppuku";
	}
	else
	{
		Response["response"] = "you ok there bud ?";
	}
	
	SendJson(Response);
}

void JsonListener::HandleResponse(const json &Response)
{

}

bool JsonListener::IsQuery(const json &object)
{
	return object.contains("query");
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
	chrono::duration<double> TimeSinceLastAliveReceived = chrono::steady_clock::now() - LastAliveReceived;
	chrono::duration<double> TimeSinceLastAliveSent = chrono::steady_clock::now() - LastAliveSent;
	if (TimeSinceLastAliveReceived.count() > 3.0)
	{
		cout << "Got no activity from " << ClientName << ", closing..." << endl;
		killed = true;
		return;
	}
	
	if (TimeSinceLastAliveReceived.count() > 1.0 && TimeSinceLastAliveSent.count() > 1.0)
	{
		cout << "Asking " << ClientName << " if it's alive..." << endl;
		LastAliveSent = chrono::steady_clock::now();
		json query;
		query["query"] = "alive";
		query["index"] = sendIndex++;
		SendJson(query);
	}
}

void JsonListener::ThreadEntryPoint()
{
	while (!killed)
	{
		CheckAlive();
		
		char bufferraw[1024];
		int numreceived = Transport->Receive(bufferraw, sizeof(bufferraw), ClientName, false);
		if (numreceived <= 0)
		{
			this_thread::sleep_for(chrono::microseconds(500));
			continue;
		}
		int rcvbufstartpos = 0;
		int rcvbufinsertpos = ReceiveBuffer.size();

		ReceiveBuffer.insert(ReceiveBuffer.end(), bufferraw, bufferraw+numreceived);
		
		for (int i = rcvbufinsertpos; i < ReceiveBuffer.size(); i++)
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