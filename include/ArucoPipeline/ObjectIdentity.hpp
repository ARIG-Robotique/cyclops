#pragma once

#include <cstdint>
#include <string>
#include <optional>
#include <vector>
#include <map>
#include <opencv2/core/affine.hpp>

enum class CDFRTeam
{
	Unknown,
	Yellow,
	Blue
};

const std::map<CDFRTeam, std::string> TeamNames = {
	{CDFRTeam::Unknown, "Unknown"},
	{CDFRTeam::Blue, "Blue"},
	{CDFRTeam::Yellow, "Yellow"}
};

enum class ObjectType
{
	Unknown,
	Tag,
	ReferenceAbsolute,
	ReferenceRelative,
	Camera,
	Object,
	Robot,
	TopTracker,
	TeamTracker,

	SolarPanel,
	Team
};

const std::map<ObjectType, std::string> ObjectTypeNames = 
{
	{ObjectType::Unknown, 			"Unknown"},
	{ObjectType::Tag, 				"Tag"},
	{ObjectType::ReferenceAbsolute, "Reference Absolute"},
	{ObjectType::ReferenceRelative, "Reference Relative"},
	{ObjectType::Camera, 			"Camera"},
	{ObjectType::Object, 			"Object"},
	{ObjectType::Robot, 			"Robot"},
	{ObjectType::TopTracker, 		"Top Tracker"},
	{ObjectType::TeamTracker, 		"Team Tracker"},

	{ObjectType::SolarPanel, 		"Solar Panel"},
	{ObjectType::Team, 				"Team"}
};

struct ObjectData
{
	ObjectType type;
	std::string name;
	std::string metadata;
	cv::Affine3d location;

	std::vector<ObjectData> Childs;

	ObjectData(ObjectType InType = ObjectType::Unknown, const std::string InName = "None", cv::Affine3d InLocation = cv::Affine3d::Identity())
		:type(InType), name(InName), location(InLocation)
	{}

	ObjectData(ObjectType InType, const std::string InName, double posX, double posY, double posZ)
		:type(InType), name(InName)
	{
		location = cv::Affine3d::Identity();
		location.translation(cv::Vec3d(posX, posY, posZ));
	}

	std::optional<struct GLObject> ToGLObject() const;

	static std::vector<GLObject> ToGLObjects(const std::vector<ObjectData>& data);
};
