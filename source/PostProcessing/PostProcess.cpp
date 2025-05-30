#include "PostProcessing/PostProcess.hpp"
#include <EntryPoints/CDFRExternal.hpp>

using namespace std;
using namespace cv;

PostProcess::PostProcess(CDFRExternal* InOwner)
	:Owner(InOwner)
{
}

PostProcess::~PostProcess()
{
}

vector<ObjectData> PostProcess::GetEnemyRobots(vector<ObjectData> &Objects) const 
{
	assert(Owner != nullptr);
	auto OurTeam = Owner->GetTeam();
	auto EnemyTeam = GetOtherTeam(OurTeam);
	const string& EnemyTeamName = TeamNames.at(EnemyTeam).JavaName;
	vector<ObjectData> robots;
	for (auto &obj : Objects)
	{
		if (obj.type != ObjectType::Robot)
		{
			continue;
		}
		if (obj.name.rfind(EnemyTeamName, 0) != 0)
		{
			continue;
		}
		robots.emplace_back(obj);
	}
	return robots;
}

void PostProcess::Reset()
{

}

void PostProcess::Process(vector<CameraImageData> &ImageData, vector<CameraFeatureData> &FeatureData, vector<ObjectData> &Objects)
{
	(void) ImageData;
	(void) FeatureData;
	(void) Objects;
}