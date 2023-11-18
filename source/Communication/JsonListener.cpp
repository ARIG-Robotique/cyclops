#include "Communication/JsonListener.hpp"
#include <Communication/Transport/TCPTransport.hpp>

#include <nlohmann/json.hpp>
#include <iostream>

using namespace std;
using namespace nlohmann;

JsonListener::JsonListener(TCPTransport* InTransport, string InClientName)
	:Transport(InTransport), ClientName(InClientName)
{
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
	
	const string &query = parsed.value("query", "invalid");
	int index = parsed.value("index", -1);
	json response;
	response["index"] = index;

	if (query == "alive")
	{
		response["response"] = true;
	}
	else if (query == "config") //idk, do something ?
	{
		
	}
	else if (query == "status") //How are the cameras doing ?
	{
		
	}
	else if (query == "data") //2D or 3D data
	{
		
	}
	else if (query == "processing") //oh no, homework !
	{

	}
	else if (query == "seppuku")
	{
		killed = true;
		response["response"] = "aight, imma commit seppuku";
	}
	else
	{
		response["response"] = "you ok there bud ?";
	}
	
	string SendBuffer = response.dump() + "\n";

	if(!Transport->Send(SendBuffer.data(), SendBuffer.length(), ClientName))
	{
		killed = true;
	}
}

void JsonListener::ThreadEntryPoint()
{
	while (!killed)
	{
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