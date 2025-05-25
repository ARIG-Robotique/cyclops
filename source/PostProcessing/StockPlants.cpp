#include <PostProcessing/StockPlants.hpp>
#include <EntryPoints/CDFRExternal.hpp>

using namespace std;
using namespace cv;

PostProcessStockPlants::PostProcessStockPlants(CDFRExternal* InOwner)
	:PostProcess(InOwner)
{
	array<string, 6> names = {"Nord Ouest", "Nord", "Nord Est", "Sud Ouest", "Sud", "Sud Est", };
	for (size_t i = 0; i < Stocks.size(); i++)
	{
		auto& stock = Stocks[i];
		auto &position = stock.position;
		bool north = i<3;
		bool center = (i%3)==1;
		bool west = (i%3)==0;
		if (center)
		{
			position = {0.000,0.500};
		}
		else
		{
			position = {0.500, 0.300};
			if (west)
			{
				position[0] *= -1;
			}
			
		}
		if (!north)
		{
			position[1] *= -1;
		}
		stock.name = names[i];
	}
	
}

void PostProcessStockPlants::Process(std::vector<CameraImageData> &ImageData, std::vector<CameraFeatureData> &FeatureData, std::vector<ObjectData> &Objects)
{
	(void) ImageData;
	(void) FeatureData;
	for (auto &zone : Stocks)
	{
		zone.NumPlants = 0;
	}
	
	for (auto &obj : Objects)
	{
		Vec2d pos2d = obj.GetPos2D();
		bool plant = obj.type == ObjectType::Fragile2024 || obj.type == ObjectType::Resistant2024;
		bool robot = obj.type == ObjectType::Robot;
		if (!robot && !plant)
		{
			continue;
		}
		for (auto &zone : Stocks)
		{
			auto delta = pos2d - zone.position;
			double distance = sqrt(delta.ddot(delta));
			if (distance < zone.radius + robot * 0.2)
			{
				if (robot)
				{
					zone.LastTouched = obj.LastSeen;
				}
				else
				{
					zone.NumPlants++;
				}
				break;
			}
		}
	}
	
	for (auto &zone : Stocks)
	{
		ObjectData obj(ObjectType::PlantStock2024, zone.name, Affine3d(Vec3d::all(0), Vec3d(zone.position[0], zone.position[1], 0)));
		obj.metadata["numPlantes"] = zone.NumPlants;
		bool intact = zone.LastTouched == ObjectData::TimePoint();
		obj.metadata["intact"] = intact;
		if (!intact)
		{
			obj.metadata["ageContact"] = chrono::duration_cast<chrono::milliseconds>(ObjectData::Clock::now() - zone.LastTouched).count();
		}
		
		Objects.push_back(obj);
	}
}