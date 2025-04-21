#pragma once

#include <memory>
#include <vector>

#include <imgui.h>
#include <Visualisation/GLWindow.hpp>
#include <Visualisation/openGL/Texture.hpp>
#include <Transport/Task.hpp>
#include <opencv2/core.hpp>

class ImguiWindow : public GLWindow
{

	static bool ImguiOpenGLInit;
	static std::string ImguiIniPath;
	std::string WindowName;


public:
	std::vector<Texture> Textures;

	ImguiWindow(std::string InWindowName = "ImGui");
	virtual ~ImguiWindow();

	virtual void WindowSizeCallback(int width, int height) override;

	ImVec2 GetWindowSize();
	
	void Init();

	void StartFrame();
	bool EndFrame();

	void AddImageToBackground(const Texture &Image, cv::Rect impos, 
		cv::Size2f UVmin = cv::Size2f(0,0), cv::Size2f UVmax = cv::Size2f(1,1));

	void AddImageToBackground(unsigned int slot, const cv::UMat &Image, cv::Rect impos, 
		cv::Size2f UVmin = cv::Size2f(0,0), cv::Size2f UVmax = cv::Size2f(1,1));


};


