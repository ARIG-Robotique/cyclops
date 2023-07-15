#pragma once

#include "imgui.h"
#include "visualisation/GLWindow.hpp"

class ImguiWindow : public GLWindow
{
public:
	ImguiWindow();
	virtual ~ImguiWindow();

	virtual void WindowSizeCallback(int width, int height) override;

	ImVec2 GetWindowSize();

	void StartFrame();
	bool EndFrame();
};


