#pragma once

#include "Cameras/Camera.hpp"

class FakeCamera : public Camera
{
public:
	FakeCamera(CameraSettings InSettings)
		:Camera(InSettings)
	{}

	virtual ~FakeCamera()
	{}

	virtual bool Grab() override;

	virtual bool Read() override;

	void Inject(cv::UMat& image);
};
