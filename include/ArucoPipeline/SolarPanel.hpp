#pragma once

#include <ArucoPipeline/TrackedObject.hpp>

#include <array>

class SolarPanel : public TrackedObject
{
private:
	std::array<double, 9> PanelRotations; //Solar panels, X increasing (so going from blue to yellow side)
public:
	SolarPanel();
};
