
#include "Visualisation/ImguiWindow.hpp"

#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <imgui.h>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <string>
#include <iostream>

using namespace std;

bool ImguiWindow::ImguiOpenGLInit = false;

ImguiWindow::ImguiWindow(string WindowName)
{
	cout << "Creating ImGui window" << endl;
	GLCreateWindow(1280, 720, WindowName);
	if (!Window)
	{
		return;
	}

	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO(); (void)io;
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;	 // Enable Keyboard Controls
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;	  // Enable Gamepad Controls
	
	// Setup Dear ImGui style
	ImGui::StyleColorsDark();
	//ImGui::StyleColorsLight();

	// Setup Platform/Renderer backends
	ImGui_ImplGlfw_InitForOpenGL(Window, true);
	if (ImguiOpenGLInit)
	{
		return;
	}
	ImguiOpenGLInit = true;
	ImGui_ImplOpenGL3_Init(nullptr);
}

ImguiWindow::~ImguiWindow()
{
	cout << "Deleting ImGui" << endl;
	ImGui_ImplGlfw_Shutdown();
}

void ImguiWindow::WindowSizeCallback(int width, int height)
{
	//glViewport(0, 0, width, height);
	ImGui::GetIO().DisplaySize = ImVec2(width, height);
}

ImVec2 ImguiWindow::GetWindowSize()
{
	if (!Window)
	{
		return ImVec2(0,0);
	}
	int winwidth, winheight;
	glfwGetWindowSize(Window, &winwidth, &winheight);
	return ImVec2(winwidth, winheight);
}

void ImguiWindow::StartFrame()
{
	if (!Window)
	{
		return;
	}
	glfwMakeContextCurrent(Window);
	// Poll and handle events (inputs, window resize, etc.)
	// You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
	// - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application, or clear/overwrite your copy of the mouse data.
	// - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application, or clear/overwrite your copy of the keyboard data.
	// Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
	glfwPollEvents();

	// Start the Dear ImGui frame
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();
}

bool ImguiWindow::EndFrame()
{
	if (!Window)
	{
		return true;
	}
	// Rendering
	ImGui::Render();
	glClearColor(0,0,0,0);
	glClear(GL_COLOR_BUFFER_BIT);
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

	glfwSwapBuffers(Window);

	bool IsDone = glfwWindowShouldClose(Window) == 0;
	glfwMakeContextCurrent(NULL);

	return IsDone;
}

void ImguiWindow::AddImageToBackground(const Texture &Image, cv::Rect impos, cv::Size2f UVmin, cv::Size2f UVmax)
{
	if(!Window)
	{
		return;
	}
	ImDrawList* background = ImGui::GetBackgroundDrawList();
	{
		ImVec2 p_min = impos.tl(), p_max = impos.br();
		
		background->AddImage((void*)(intptr_t)Image.GetTextureID(), p_min, p_max, ImVec2(UVmin), ImVec2(UVmax));
	}
}