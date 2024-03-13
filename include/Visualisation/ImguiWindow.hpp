#pragma once

#include <imgui.h>
#include <Visualisation/GLWindow.hpp>
#include <Visualisation/openGL/Texture.hpp>
#include <opencv2/core.hpp>

class ImguiWindow : public GLWindow
{

	static bool ImguiOpenGLInit;
public:
	ImguiWindow();
	virtual ~ImguiWindow();

	virtual void WindowSizeCallback(int width, int height) override;

	ImVec2 GetWindowSize();

	void StartFrame();
	bool EndFrame();

	void AddImageToBackground(const Texture &Image, cv::Rect impos, 
		cv::Size2f UVmin = cv::Size2f(0,0), cv::Size2f UVmax = cv::Size2f(1,1));
};


