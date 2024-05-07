#pragma once

#include <string>
#include <optional>
#include <vector>
#include <map>
#include <chrono>
#include <opencv2/core/affine.hpp>
#include <nlohmann/json.hpp>

enum class CDFRTeam
{
	Unknown,
	Yellow,
	Blue
};


const std::map<CDFRTeam, std::string> TeamNames = {
	{CDFRTeam::Unknown, "Unknown"},
	{CDFRTeam::Blue, "Bleu"},
	{CDFRTeam::Yellow, "Jaune"}
};

std::ostream& operator << (std::ostream& out, CDFRTeam Team);

enum class ObjectType
{
	Unknown,
	Tag,
	ReferenceAbsolute,
	ReferenceRelative,
	Camera,
	Object,
	Robot,
	Pami,

	SolarPanel,

	Fragile,
	Resistant,
	Pot,
	PottedPlant,

	PlantStock,
	Hangar,
	Jardiniere,

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
	{ObjectType::Pami, 				"PAMI"},

	{ObjectType::SolarPanel, 		"Solar panel"},

	{ObjectType::Fragile, 			"Yolo3d"},
	{ObjectType::Resistant, 		"Yolo3d"},
	{ObjectType::Pot, 				"Yolo3d"},
	{ObjectType::PottedPlant, 		"Yolo3d"},

	{ObjectType::PlantStock,		"PlantStock"},
	{ObjectType::Hangar,			"Hangar"},
	{ObjectType::Jardiniere,		"Jardiniere"},

	{ObjectType::Team, 				"Team"}
};

std::ostream& operator << (std::ostream& out, ObjectType Type);

struct ObjectData
{
	typedef std::chrono::steady_clock Clock;
	typedef Clock::time_point TimePoint;
	ObjectType type;
	std::string name;
	cv::Affine3d location;
	TimePoint LastSeen;
	nlohmann::json metadata;

	std::vector<ObjectData> Childs;

	ObjectData(ObjectType InType = ObjectType::Unknown, const std::string InName = "None", 
		cv::Affine3d InLocation = cv::Affine3d::Identity(), TimePoint InLastSeen = Clock::now())
		:type(InType), name(InName), location(InLocation), LastSeen(InLastSeen)
	{}

	ObjectData(ObjectType InType, const std::string InName, double posX, double posY, double posZ)
		:type(InType), name(InName)
	{
		location = cv::Affine3d::Identity();
		location.translation(cv::Vec3d(posX, posY, posZ));
	}

	std::optional<struct GLObject> ToGLObject() const;

	static std::vector<GLObject> ToGLObjects(const std::vector<ObjectData>& data, Clock::duration maxAge = std::chrono::milliseconds(500));
};
