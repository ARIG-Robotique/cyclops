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

CDFRTeam GetOtherTeam(CDFRTeam InTeam)
{
	switch (InTeam)
	{
	case CDFRTeam::Blue:
		return CDFRTeam::Yellow;
	case CDFRTeam::Yellow:
		return CDFRTeam::Blue;
	default:
		return CDFRTeam::Unknown;
	}
}

const std::map<CDFRTeam, std::string> TeamNames = {
	{CDFRTeam::Unknown, "Unknown"},
	{CDFRTeam::Blue, "Bleu"},
	{CDFRTeam::Yellow, "Jaune"}
};

std::ostream& operator << (std::ostream& out, CDFRTeam Team);

enum class ObjectType
{
	//meta types
	Unknown,
	All,
	Data3D,
	Data2D,
	Aruco,
	Yolo,

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
	{ObjectType::All, 				"All"},
	{ObjectType::Data2D, 			"Data2D"},
	{ObjectType::Data3D, 			"Data3D"},
	{ObjectType::Aruco, 			"Aruco"},
	{ObjectType::Yolo, 				"Yolo"},

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

	cv::Vec2d GetPos2D() const
	{
		return cv::Vec2d(location.translation().val);
	}
};
