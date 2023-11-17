#include "Communication/TCPJsonHost.hpp"

#include <iostream>

#include <Communication/JsonListener.hpp>
#include <Communication/Transport/TCPTransport.hpp>

using namespace std;

void TCPJsonHost::ThreadEntryPoint(GenericTransport::NetworkInterface interface)
{
	unique_ptr<TCPTransport> Transport = make_unique<TCPTransport>(true, "0.0.0.0", Port, interface.name);
	set<unique_ptr<JsonListener>> Listeners;
	while (!killed)
	{
		auto newconnections = Transport->AcceptNewConnections();
		for (auto &&connection : newconnections)
		{
			JsonListener* listener = new JsonListener(Transport.get(), connection);
			Listeners.emplace(listener);
		}
		for (auto it = Listeners.begin(); it != Listeners.end();)
		{
			if ((*it)->IsKilled())
			{
				it=Listeners.erase(it);
			}
			else
			{
				it++;
			}
		}
		this_thread::sleep_for(chrono::milliseconds(10));
	}
}

TCPJsonHost::TCPJsonHost(int InPort)
	:Port(InPort)
{
	auto interfaces = GenericTransport::GetInterfaces();
	cout << "Listing interfaces :" << endl;
	int chosen = -1;
	for (int i=0; i<interfaces.size(); i++)
	{
		auto& ni = interfaces[i];
		cout << "\t- " << ni.name << " / IP:" << ni.address << " / Netmask:" << ni.mask << " / Broadcast:" << ni.broadcast << endl;
		if (ni.name != "lo")
		{
			thread* sendthread = new thread(&TCPJsonHost::ThreadEntryPoint, this, ni);
			ThreadHandles.emplace(sendthread);
		}
	}
	
}

TCPJsonHost::~TCPJsonHost()
{
	killed = true;
	for (auto thread : ThreadHandles)
	{
		thread->join();
		delete thread;
	}
	
}
