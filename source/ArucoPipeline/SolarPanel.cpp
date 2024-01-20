

#include <ArucoPipeline/SolarPanel.hpp>

SolarPanel::SolarPanel()
    :TrackedObject()
{
    markers = {
        ArucoMarker(37.5/1000.0, 47)
    };
}