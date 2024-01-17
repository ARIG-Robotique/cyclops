#pragma once

#include <Cameras/CameraManager.hpp>

class CameraManagerSimulation : public CameraManager
{
private:
    std::string ScenarioPath;
public:
    CameraManagerSimulation(std::string InScenarioPath)
        :ScenarioPath(InScenarioPath), CameraManager()
    {

    }

    virtual ~CameraManagerSimulation()
    {

    }

protected:
    virtual void ScanWorker() override;
};
