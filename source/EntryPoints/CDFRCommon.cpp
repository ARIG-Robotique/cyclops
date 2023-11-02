#include "EntryPoints/CDFRCommon.hpp"
#include "Misc/ManualProfiler.hpp"

map<PacketType, bool> GetDefaultAllowMap()
{
	map<PacketType, bool> allowmap = {
		{PacketType::Null, false},
		{PacketType::Camera, false},
		{PacketType::ReferenceAbsolute, false},
		{PacketType::ReferenceRelative, true},
		{PacketType::Robot, true},
		{PacketType::TeamTracker, true},
		{PacketType::TopTracker, true},
		{PacketType::Tag, false},
		{PacketType::Team, true}
	};
	return allowmap;
}