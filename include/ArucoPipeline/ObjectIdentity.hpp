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


struct TeamNameStruct
{
	std::string Name;
	std::string JavaName;
};

CDFRTeam GetOtherTeam(CDFRTeam InTeam);



const std::map<CDFRTeam, TeamNameStruct> TeamNames = {
	{CDFRTeam::Unknown, {"Unknown", "INCONNU"}},
	{CDFRTeam::Blue, 	{"Blue", 	"BLEU"}},
	{CDFRTeam::Yellow, 	{"Yellow", 	"JAUNE"}}
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

struct ObjectTypeConfig
{
	std::string Name;
	std::string JavaName;
	bool WantPosition;
	bool WantRotation;
	bool Sendable;
};


const std::map<ObjectType, ObjectTypeConfig> ObjectTypeNames =
{
	{ObjectType::Unknown, 			{"Unknown",				"INCONNU"				,0	,0	,0}},
	{ObjectType::All, 				{"All",					"TOUT"					,0	,0	,0}},
	{ObjectType::Data2D, 			{"Data2D",				"DATA2D"				,0	,0	,0}},
	{ObjectType::Data3D, 			{"Data3D",				"DATA3D"				,0	,0	,0}},
	{ObjectType::Aruco, 			{"Aruco",				"ARUCO"					,0	,0	,0}},
	{ObjectType::Yolo, 				{"Yolo",				"YOLO"					,0	,0	,0}},
	{ObjectType::Tag, 				{"Tag",					"TAG"					,1	,1	,1}},
	{ObjectType::ReferenceAbsolute, {"Reference Absolute",	"REFERENCE_ABSOLUE"		,0	,0	,0}},
	{ObjectType::ReferenceRelative, {"Reference Relative",	"REFERENCE_RELATIVE"	,1	,1	,1}},
	{ObjectType::Camera, 			{"Camera",				"CAMERA"				,1	,1	,0}},
	{ObjectType::Object, 			{"Object",				"OBJET"					,1	,1	,1}},
	{ObjectType::Robot, 			{"Robot",				"ROBOT"					,1	,1	,1}},
	{ObjectType::Pami, 				{"PAMI",				"PAMI"					,1	,1	,1}},
	{ObjectType::SolarPanel, 		{"Solar panel",			"PANNEAU_SOLAIRE"		,0	,1	,1}},
	{ObjectType::Fragile, 			{"Fragile",				"YOLO3D"				,1	,0	,1}},
	{ObjectType::Resistant, 		{"Resistant",			"YOLO3D"				,1	,0	,1}},
	{ObjectType::Pot, 				{"Pot",					"YOLO3D"				,1	,0	,1}},
	{ObjectType::PottedPlant, 		{"Potted plant",		"YOLO3D"				,1	,0	,1}},
	{ObjectType::PlantStock,		{"PlantStock",			"STOCK_PLANTES"			,0	,0	,1}},
	{ObjectType::Hangar,			{"Hangar",				"HANGAR"				,0	,0	,1}},
	{ObjectType::Jardiniere,		{"Jardiniere",			"JARDINIERE"			,0	,0	,1}},
	{ObjectType::Team, 				{"Team",				"EQUIPE"				,0	,0	,0}}
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
