#include <Communication/JsonListener.hpp>
#include <Communication/Transport/TCPTransport.hpp>

#include <nlohmann/json.hpp>
#include <iostream>

using namespace std;
using namespace nlohmann;

void JsonListener::StartListenThread()
{
	if (ListenThread)
	{
		return;
	}
	ListenThread = new thread(&JsonListener::ThreadEntryPoint, this);
}

JsonListener::~JsonListener()
{
	killed = true;
	if (ListenThread)
	{
		ListenThread->join();
	}
}

void JsonListener::ThreadEntryPoint()
{
	while (!killed)
	{
		char bufferraw[1024];
		int numreceived = Transport->Receive(bufferraw, sizeof(bufferraw), ClientName, true);
		ReceiveBuffer.write(bufferraw, numreceived);
		
		json parsed;
		try
		{
			ReceiveBuffer >> parsed;
		}
		catch(const json::exception& e)
		{
			std::cerr << e.what() << '\n';
			continue;
		}
		
		
		if (parsed.is_discarded())
		{
			continue;
		}
		
		const string &query = parsed.value("query", "invalid");
		int index = parsed.value("index", -1);
		json response;
		response.at("index") = index;

		if (query == "alive")
		{
			response.at("response") = true;
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
			response.at("response") = "aight, imma commit seppuku";
		}
		else
		{
			response.at("response") = "you ok there bud ?";
		}
		
		string SendBuffer = response.dump();

		if(!Transport->Send(SendBuffer.data(), SendBuffer.length(), ClientName))
		{
			killed = true;
		}
	}
}