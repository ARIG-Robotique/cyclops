#pragma once

#include <Cameras/CameraManager.hpp>

class CameraManagerSimulation : public CameraManager
{
private:
	std::string ScenarioPath;
public:
	CameraManagerSimulation(std::string InScenarioPath)
		:CameraManager(), ScenarioPath(InScenarioPath)
	{

	}

	virtual ~CameraManagerSimulation()
	{

	}

protected:
	virtual void ThreadEntryPoint() override;
};
