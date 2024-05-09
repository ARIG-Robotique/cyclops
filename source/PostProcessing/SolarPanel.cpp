#include <PostProcessing/SolarPanel.hpp>
#include <Misc/math3d.hpp>

using namespace std;

void PostProcessSolarPanel::Process(std::vector<CameraImageData> &ImageData, std::vector<CameraFeatureData> &FeatureData, std::vector<ObjectData> &Objects)
{
	(void) ImageData;
	(void) FeatureData;
	const int lowthreshold = 5, highthreshold = 155;
	static const array<string, 4> teams = {"AUCUNE", "JAUNE", "BLEU", "JAUNE_ET_BLEU"};
	for (auto &obj : Objects)
	{
		if (obj.type == ObjectType::SolarPanel)
		{
			
			double rotZ = GetRotZ(obj.location.rotation());
			double rotZdeg = rotZ*180.0/M_PI;
			bool teamyellow = false, teamblue = false;
			if (rotZdeg > lowthreshold)
			{
				teamyellow = true;
			}
			if (rotZdeg < -lowthreshold)
			{
				teamblue = true;
			}
			if (rotZdeg > highthreshold)
			{
				teamblue = true;
			}
			if (rotZdeg < -highthreshold)
			{
				teamyellow = true;
			}
			obj.metadata["team"] = teams[teamblue*2+teamyellow];
		}
	}
}