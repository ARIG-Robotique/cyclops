#include <PostProcessing/ZoneWatcher.hpp>
#include <EntryPoints/CDFRExternal.hpp>

#include <iostream>

using namespace std;
using namespace cv;

Rect2d InkscapeToRect(double tlx, double tly, bool vertical, Size2d extent)
{
	double &width = extent.width, &height = extent.height;
	double dx = (vertical ? height/2 : width/2);
	double dy = (vertical ? width/2 : height/2);
	double cx = tlx - 1.5 + dx;
	double cy = 1-tly - dy;

	return Rect2d(Point2d(cx-dx, cy-dy), Size2d(dx*2,dy*2));
}

PostProcessZone::PostProcessZone(CDFRExternal* InOwner)
	:PostProcess(InOwner)
{
	Reset();
}

void PostProcessZone::Reset()
{
	Size2d StockExtent(0.375, 0.075), BigZone(0.450, 0.450), SmallZone(0.450, 0.150);
	array<Rect2d, 5> PositionsStocks = {
		InkscapeToRect(1.9875, 0.2375, false, StockExtent),	//Nord Est
		InkscapeToRect(2.8875, 0.4875, true, StockExtent),	//Est Nord
		InkscapeToRect(1.7125, 1.0125, false, StockExtent),	//Centre Est
		InkscapeToRect(2.8875, 1.4215, true, StockExtent),	//Est Sud
		InkscapeToRect(2.0375, 1.7125, false, StockExtent)	//Sud Est
	};
	array<Rect2d, 4> PositionsBlueZones = {
		InkscapeToRect(1.55, 1.55, false, BigZone),
		InkscapeToRect(0, 0.9, false, BigZone),
		InkscapeToRect(2, 1.85, false, SmallZone),
		InkscapeToRect(0, 1.85, false, SmallZone)
	};
	array<string, 18> names = {"Bleu reserve", "Bleu haut droite", "Bleu milieu centre", "Bleu bas droite", "Bleu bas centre", 
		"Jaune reserve", "Jaune haut gauche", "Jaune milieu centre", "Jaune bas gauche", "Jaune bas centre", 
		"Gros bleu centre", "Gros bleu gauche", "Petit bleu droite", "Petit bleu gauche",
		"Gros jaune centre", "Gros jaune droite", "Petit jaune gauche", "Petit jaune droite"
	};
	assert(names.size() == Zones.size());
	for (size_t i = 0; i < Zones.size(); i++)
	{
		auto& stock = Zones[i];
		auto &position = stock.position;
		bool isstock = i<10;
		bool flip;
		if (isstock)
		{
			flip = i >= PositionsStocks.size();
			stock.position = PositionsStocks[i%PositionsStocks.size()];
			stock.radius = 0.25;
		}
		else
		{
			size_t newidx = i-PositionsStocks.size()*2;
			flip = newidx>=PositionsBlueZones.size();
			stock.position = PositionsBlueZones[newidx%PositionsBlueZones.size()];
			stock.radius = 0.2;
		}
		
		
		if (flip)
		{
			position.x = -position.x-position.width;
		}
		stock.LastContactStart = ObjectData::TimePoint();
		stock.LastContactEnd = ObjectData::TimePoint();
		stock.TimeSpentContacting = chrono::seconds(0);
		stock.Contacting = false;
		stock.ContactThisTick = false;
		stock.IsStock = isstock;
		stock.name = names[i];
	}
}

void PostProcessZone::Process(std::vector<CameraImageData> &ImageData, std::vector<CameraFeatureData> &FeatureData, std::vector<ObjectData> &Objects)
{
	(void) ImageData;
	(void) FeatureData;

	for (auto &zone : Zones)
	{
		zone.ContactThisTick = false;
	}
	
	
	for (auto &obj : Objects)
	{
		Vec2d pos2d = obj.GetPos2D();
		bool robot = obj.type == ObjectType::Robot;
		if (!robot)
		{
			continue;
		}
		for (auto &zone : Zones)
		{
			auto tl = zone.position.tl();
			auto br = zone.position.br();
			Vec2d PosInRect(clamp<double>(pos2d[0], tl.x, br.x), clamp<double>(pos2d[1], tl.y, br.y));
			auto delta = pos2d - PosInRect;
			double distance = sqrt(delta.ddot(delta));
			if (distance < zone.radius)
			{
				if (robot)
				{
					if (!zone.Contacting)
					{
						zone.LastContactStart = obj.LastSeen;
					}
					
					zone.ContactThisTick = true;
					zone.LastContactEnd = obj.LastSeen;
					cout << "Zone " << zone.name << " was just touched!" <<endl;
				}
				break;
			}
		}
	}
	
	for (auto &zone : Zones)
	{
		if (zone.Contacting && !zone.ContactThisTick)
		{
			zone.TimeSpentContacting += zone.LastContactEnd - zone.LastContactStart; 
			zone.Contacting = false;
		}
		if(!zone.Contacting && zone.ContactThisTick)
		{
			zone.Contacting = true;
		}
		
		ObjectData obj(zone.IsStock ? ObjectType::Stock2025 : ObjectType::DropZone2025, zone.name, 
			Affine3d(Vec3d::all(0), Vec3d(zone.position.x+zone.position.width/2, zone.position.y+zone.position.height/2, 0)));
		bool intact = zone.LastContactEnd == ObjectData::TimePoint();
		obj.metadata["intact"] = intact;
		if (!intact)
		{
			obj.metadata["lastContactStartAge"] = chrono::duration_cast<chrono::milliseconds>(ObjectData::Clock::now() - zone.LastContactStart).count();
			obj.metadata["lastContactEndAge"] = chrono::duration_cast<chrono::milliseconds>(ObjectData::Clock::now() - zone.LastContactEnd).count();
		}
		auto time_spent_near = zone.TimeSpentContacting + (zone.Contacting ? zone.LastContactEnd - zone.LastContactStart : chrono::seconds(0));
		obj.metadata["timeSpentNear"] = chrono::duration_cast<chrono::milliseconds>(time_spent_near).count();
		obj.metadata["contacting"] = zone.Contacting;
		obj.metadata["somethingHigh"] = nlohmann::json();
		
		Objects.push_back(obj);
	}
}