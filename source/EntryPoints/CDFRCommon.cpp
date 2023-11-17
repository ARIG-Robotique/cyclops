#include "EntryPoints/CDFRCommon.hpp"
#include <Misc/ManualProfiler.hpp>

map<ObjectType, bool> GetDefaultAllowMap()
{
	map<ObjectType, bool> allowmap = {
		{ObjectType::Unknown, false},
		{ObjectType::Camera, false},
		{ObjectType::ReferenceAbsolute, false},
		{ObjectType::ReferenceRelative, true},
		{ObjectType::Robot, true},
		{ObjectType::TeamTracker, true},
		{ObjectType::TopTracker, true},
		{ObjectType::Tag, false},
		{ObjectType::Team, true}
	};
	return allowmap;
}