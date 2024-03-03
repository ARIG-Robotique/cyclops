#include "Communication/TCPJsonHost.hpp"

#include <iostream>

#include <Communication/JsonListener.hpp>
#include <Communication/Transport/TCPTransport.hpp>
#include <EntryPoints/CDFRExternal.hpp>

using namespace std;

void TCPJsonHost::ThreadEntryPoint(GenericTransport::NetworkInterface interface)
{
	unique_ptr<TCPTransport> Transport = make_unique<TCPTransport>(true, "0.0.0.0", Port, interface.name);
	set<shared_ptr<JsonListener>> Listeners;
	while (!killed)
	{
		auto newconnections = Transport->AcceptNewConnections();
		for (auto &&connection : newconnections)
		{
			JsonListener* listener = new JsonListener(Transport.get(), connection, this);
			Listeners.emplace(listener);
			NumClients++;
			ExternalRunner->SetHasNoClients(NumClients == 0);
			cout << NumClients << " clients right now" << endl;
		}
		for (auto it = Listeners.begin(); it != Listeners.end();)
		{
			shared_ptr<JsonListener> lptr = *it;
			if (lptr->IsKilled())
			{
				cout << "Client at " << lptr->ClientName << " is killed, cleaning..." << endl;
				Transport->DisconnectClient(lptr->ClientName);
				it=Listeners.erase(it);
				NumClients--;
				ExternalRunner->SetHasNoClients(NumClients == 0);
				cout << NumClients << " clients right now" << endl;
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
	for (size_t i=0; i<interfaces.size(); i++)
	{
		auto& ni = interfaces[i];
		
		cout << "Starting TCP Json host on " << ni.name << " / IP:" << ni.address << " / Netmask:" << ni.mask << " / Broadcast:" << ni.broadcast << endl;
		thread* sendthread = new thread(&TCPJsonHost::ThreadEntryPoint, this, ni);
		ThreadHandles.emplace(sendthread);
	}
}

TCPJsonHost::~TCPJsonHost()
{
	killed = true;
	for (auto &thread : ThreadHandles)
	{
		thread->join();
	}
	
}
