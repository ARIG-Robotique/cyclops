#include "Communication/TCPJsonHost.hpp"

#include <iostream>

#include <Communication/JsonListener.hpp>
#include <Transport/TCPTransport.hpp>
#include <Transport/ConnectionToken.hpp>
#include <EntryPoints/CDFRExternal.hpp>
#include <Transport/thread-rename.hpp>

using namespace std;

void TCPJsonHost::ThreadEntryPoint(optional<GenericTransport::NetworkInterface> interface)
{
	string ThreadName = interface.has_value() ? string("TCP Json Host on ") + interface.value().name : string("TCP Json Host");
	SetThreadName(ThreadName.c_str());
	
	
	unique_ptr<TCPTransport> Transport = make_unique<TCPTransport>(true, "0.0.0.0", Port, interface.has_value() ? interface.value().name : "");
	set<shared_ptr<JsonListener>> Listeners;
	while (!killed)
	{
		auto newconnections = Transport->AcceptNewConnections();
		for (auto &&connection : newconnections)
		{
			JsonListener* listener = new JsonListener(connection, this);
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
				cout << "Client at " << lptr->token->GetConnectionName() << " is killed, cleaning..." << endl;
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
		
		this_thread::sleep_for(chrono::milliseconds(100));
	}
}

TCPJsonHost::TCPJsonHost(int InPort)
	:Port(InPort)
{
	auto interfaces = GenericTransport::GetInterfaces();
	#if 0
	for (size_t i=0; i<interfaces.size(); i++)
	{
		auto& ni = interfaces[i];
		
		cout << "Starting TCP Json host on " << ni.name << " / IP:" << ni.address << " / Netmask:" << ni.mask << " / Broadcast:" << ni.broadcast << endl;
		thread* sendthread = new thread(&TCPJsonHost::ThreadEntryPoint, this, ni);
		ThreadHandles.emplace(sendthread);
	}
	#else
	cout << "Starting TCP Json host, not bound to a specific interface" << endl;
	thread* sendthread = new thread(&TCPJsonHost::ThreadEntryPoint, this, nullopt);
	ThreadHandles.emplace(sendthread);
	#endif
}

TCPJsonHost::~TCPJsonHost()
{
	killed = true;
	for (auto &thread : ThreadHandles)
	{
		thread->join();
	}
}
