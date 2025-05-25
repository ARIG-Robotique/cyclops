#include "Visualisation/BoardGL.hpp"

#include <iostream>
#include <tuple>
#include <optional>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>	

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>
#include <assimp/Importer.hpp>

#include <ArucoPipeline/ArucoTypes.hpp>
#include <Misc/path.hpp>
#include <Misc/GlobalConf.hpp>
#include <Visualisation/openGL/Mesh.hpp>

using namespace std;


GLObject::GLObject(MeshNames InType, double x, double y, double z, std::string InMetadata)
{
	type = InType;
	location = glm::translate(glm::mat4(1), {x,y,z});
	metadata = InMetadata;
}

BoardGL::BoardGL(string InName)
	:GLWindow(), name(InName)
{

}

BoardGL::~BoardGL()
{
	for (auto &&i : Meshes)
	{
		i.second.Release();
	}
	for (auto &&i : TagTextures)
	{
		i.Release();
	}
}

void BoardGL::WindowSizeCallback(int width, int height)
{
	glViewport(0, 0, width, height);
}

glm::mat4 BoardGL::GetVPMatrix(glm::vec3 forward, glm::vec3 up) const
{
	int winwidth, winheight;
	glfwGetWindowSize(Window, &winwidth, &winheight);

	glm::mat4 CameraMatrix = glm::lookAt(
		cameraPosition,
		cameraPosition+forward,
		up
	);

	glm::mat4 projectionMatrix = glm::perspective(
		glm::radians(FoV),						// The vertical Field of View, in radians
		(float) winwidth / (float)winheight,	//Aspect ratio
		0.01f,									// Near clipping plane.
		200.0f									// Far clipping plane.
	);

	return projectionMatrix * CameraMatrix;
}

glm::vec3 BoardGL::GetDirection() const
{
	// Direction : Spherical coordinates to Cartesian coordinates conversion
	double sinh, cosh;
	sincos(horizontalAngle, &sinh, &cosh);
	double sinv, cosv;
	sincos(verticalAngle, &sinv, &cosv);
	glm::vec3 direction(
		cosv * sinh,
		cosv * cosh,
		sinv
	);
	return direction;
}

glm::vec3 BoardGL::GetRightVector() const
{
	// Right vector
	double sinh, cosh;
	sincos(horizontalAngle, &sinh, &cosh);
	glm::vec3 right = glm::vec3(
		cosh,
		-sinh,
		0
	);
	return right;
}




void BoardGL::HandleInputs()
{
	double currentTime = glfwGetTime();
	float deltaTime = float(currentTime - lastTime);
	int winwidth, winheight;
	glfwGetWindowSize(Window, &winwidth, &winheight);

	if (glfwGetMouseButton(Window, GLFW_MOUSE_BUTTON_LEFT))
	{
		// Get mouse position
		double xpos, ypos;
		if (LookingAround)
		{
			glfwGetCursorPos(Window, &xpos, &ypos);
		}
		else
		{
			LookingAround = true;
			xpos = winwidth/2.0;
			ypos = winheight/2.0;
			glfwSetInputMode(Window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
		}
		
		glfwSetCursorPos(Window, winwidth/2.0, winheight/2.0);
		// Compute new orientation
		double xdiff = winwidth/2.0 - xpos, ydiff = winheight/2.0 - ypos;
		horizontalAngle -= mouseSpeed * xdiff;
		verticalAngle 	+= mouseSpeed * ydiff;
		//cout << "X: " << xdiff << " Y :" << ydiff << " h: " << horizontalAngle << " v: " << verticalAngle << endl;
		
	}
	else
	{
		LookingAround = false;
		glfwSetInputMode(Window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
	}

	const double LookSensitivty = 0.5;
	// Look down
	if (glfwGetKey(Window, GLFW_KEY_KP_2) == GLFW_PRESS){
		verticalAngle -= deltaTime * LookSensitivty;
	}
	// Look up
	if (glfwGetKey(Window, GLFW_KEY_KP_8) == GLFW_PRESS){
		verticalAngle += deltaTime * LookSensitivty;
	}
	// Look right
	if (glfwGetKey(Window, GLFW_KEY_KP_6) == GLFW_PRESS){
		horizontalAngle += deltaTime * LookSensitivty;
	}
	// Look left
	if (glfwGetKey(Window, GLFW_KEY_KP_4) == GLFW_PRESS){
		horizontalAngle -= deltaTime * LookSensitivty;
	}
	

	verticalAngle = clamp<float>(verticalAngle, -M_PI_2, M_PI_2);
	horizontalAngle = fmod(horizontalAngle, M_PI*2);

	glm::vec3 direction = GetDirection();
	glm::vec3 right = GetRightVector();
	glm::vec3 up = glm::cross(right, direction);

	float speed = 1;
	// Move forward
	if (glfwGetKey(Window, GLFW_KEY_UP) == GLFW_PRESS){
		cameraPosition += direction * deltaTime * speed;
	}
	// Move backward
	if (glfwGetKey(Window, GLFW_KEY_DOWN) == GLFW_PRESS){
		cameraPosition -= direction * deltaTime * speed;
	}
	// Strafe right
	if (glfwGetKey(Window, GLFW_KEY_RIGHT) == GLFW_PRESS){
		cameraPosition += right * deltaTime * speed;
	}
	// Strafe left
	if (glfwGetKey(Window, GLFW_KEY_LEFT) == GLFW_PRESS){
		cameraPosition -= right * deltaTime * speed;
	}
	// Move up
	if (glfwGetKey(Window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS){
		cameraPosition += up * deltaTime * speed;
	}
	// Move down
	if (glfwGetKey(Window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS){
		cameraPosition -= up * deltaTime * speed;
	}

	//cout << "pos: " << cameraPosition.x << " " << cameraPosition.y << " " << cameraPosition.z << endl;

	lastTime = currentTime;
}

void BoardGL::LoadModels()
{
	if (!Window)
	{
		return;
	}
	if (MeshesLoaded)
	{
		return;
	}
	glfwMakeContextCurrent(Window);
	auto assetspath = GetAssetsPath() / "models";
	static const map<MeshNames, string> meshpathes = 
	{
		{MeshNames::arena,			"board"},
		{MeshNames::robot,			"robot"},
		{MeshNames::axis,			"axis"},
		{MeshNames::camera,			"camera"},
		{MeshNames::skybox,			"skybox"},
		{MeshNames::tag, 			"tag"},
		{MeshNames::trackercube,	"tracker"},
		{MeshNames::toptracker,		"top tracker"},

		#if 0
		{MeshNames::solarpanel2024,		"2024/solar panel"},
		{MeshNames::fragile2024,		"2024/fragile"},
		{MeshNames::resistant2024,		"2024/resistant"},
		{MeshNames::pot2024,			"2024/pot"},
		{MeshNames::potted_plant2024,	"2024/potted plant"},
		#endif

		#if 1
		{MeshNames::plank2025,		"2025/plank"},
		{MeshNames::can2025,		"2025/can"},

		#endif

	};
	//cout << "Loading meshes" << endl;

	for (auto iterator : meshpathes)
	{
		Meshes[iterator.first] = Mesh();
		Mesh &thismesh = Meshes[iterator.first];
		filesystem::path folderpath = assetspath / iterator.second;
		if (!filesystem::exists(folderpath))
		{
			cerr << "[BoardGL::LoadModels] " << folderpath << " does not exist" << endl;
			continue;
		}
		optional<filesystem::path> meshpath, texturepath;
		for (auto &i : std::filesystem::directory_iterator(folderpath))
		{
			//cout << "Found " << i << endl;
			if (!i.is_regular_file())
			{
				continue;
			}
			auto entry = i.path();
			if (entry.extension() == ".obj")
			{
				meshpath = entry;
			}
			else if (entry.extension() == ".png" || entry.extension() == ".jpeg")
			{
				texturepath = entry;
			}
		}
		if (!meshpath.has_value())
		{
			cerr << "[BoardGL::LoadModels] Found no mesh for " << folderpath << endl;
			continue;
		}
		if (texturepath.has_value())
		{
			thismesh.LoadFromFile(meshpath.value(), texturepath.value());
		}
		else
		{
			thismesh.LoadFromFile(meshpath.value());
		}
		thismesh.BindMesh();
	}
	MeshesLoaded = true;
	glfwMakeContextCurrent(NULL);
}

void BoardGL::LoadTags()
{
	if (!Window)
	{
		return;
	}
	if (TagsLoaded)
	{
		return;
	}
	glfwMakeContextCurrent(Window);

	TagTextures.resize(ARUCO_DICT_SIZE);
	auto& det = GetArucoDetector();
	auto& dict = det.getDictionary();
	for (int i = 0; i < ARUCO_DICT_SIZE; i++)
	{
		cv::Mat texture;
		cv::aruco::generateImageMarker(dict, i, 128, texture, 1);
		cv::cvtColor(texture, TagTextures[i].SourceImage, cv::COLOR_GRAY2BGR);
		//cv::imshow("Aruco SourceImage", TagTextures[i].Texture);
		//cv::waitKey();
		TagTextures[i].valid = true;
		TagTextures[i].Bind();
	}
	TagsLoaded = true;
	glfwMakeContextCurrent(NULL);
}

void BoardGL::Init()
{
	GLCreateWindow(DoScreenCapture() ? 1920 : 1280, DoScreenCapture() ? 1080 : 720, name);
	if (DoScreenCapture())
	{
		try
		{
			filesystem::remove_all(GetScreenCapturePath()/"3D");
		}
		catch(const std::exception& e)
		{
			std::cerr << "Screen capture 3D remove_all: " << e.what() << '\n';
		}
		filesystem::create_directories(GetScreenCapturePath()/"3D");
	}
	
	if (!Window)
	{
		return;
	}

	// Enable depth test
	glEnable(GL_DEPTH_TEST);
	// Cull triangles which normal is not towards the camera
	glEnable(GL_CULL_FACE);
	// Accept fragment if it closer to the camera than the former one
	glDepthFunc(GL_LESS);

	glfwSetInputMode(Window, GLFW_STICKY_KEYS, GL_TRUE);
	glfwSetInputMode(Window, GLFW_RAW_MOUSE_MOTION, GL_TRUE);
	
	auto shaderfolder = GetAssetsPath() / "shaders";
	// Create and compile our GLSL program from the shaders
	ShaderProgram.LoadShader(shaderfolder / "vertexshader.vs", shaderfolder / "fragmentshader.fs");

	glGenVertexArrays(1, &VertexArrayID);
	glBindVertexArray(VertexArrayID);
	
	glfwMakeContextCurrent(NULL);

	LoadModels();
	//cout << "OpenGL init done!" << endl;
}

bool BoardGL::Tick(std::vector<GLObject> data)
{
	if (!Window)
	{
		return true;
	}
	glfwMakeContextCurrent(Window);
	glfwPollEvents();
	HandleInputs();
	glm::vec3 direction = GetDirection();
	glm::vec3 right = GetRightVector();

	// Up vector : perpendicular to both direction and right
	glm::vec3 up = glm::cross(right, direction);

	glm::mat4 VPMatrix = GetVPMatrix(direction, up);

	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glUseProgram(ShaderProgram.ProgramID);

	GLuint MatrixID = glGetUniformLocation(ShaderProgram.ProgramID, "MVP"); //projection matrix handle
	GLuint TextureSamplerID  = glGetUniformLocation(ShaderProgram.ProgramID, "TextureSampler"); //texture sampler handle
	glUniform1i(TextureSamplerID, 0); //set texture sampler to use texture 0
	GLuint ParameterID = glGetUniformLocation(ShaderProgram.ProgramID, "Parameters");
	GLuint ScaleID = glGetUniformLocation(ShaderProgram.ProgramID, "scale");


	glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &VPMatrix[0][0]);
	glUniform1f(ScaleID, 1);
	Meshes[MeshNames::axis].Draw(ParameterID);
	Meshes[MeshNames::skybox].Draw(ParameterID);

	

	for (size_t i = 0; i < data.size(); i++)
	{
		auto &odata = data[i];

		glm::mat4 MVPMatrix = VPMatrix * odata.location;

		glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVPMatrix[0][0]);
		glUniform1f(ScaleID, 1);
		switch (odata.type)
		{
		case MeshNames::unknown:
			break;
		case MeshNames::tag :
			{
				if (!TagsLoaded)
				{
					if (!TagLoadWarning)
					{
						cerr << "WARNING Tried to display tags but tags aren't loaded" << endl;
						TagLoadWarning = true;
					}
					break;
				}
				int number = odata.metadata.at("number"); float scale = odata.metadata.at("sideLength");
				if (number >= (int)TagTextures.size() || number < 0)
				{
					cerr << "Tried to display tag #" << number << " !" <<endl;
					break;
				}
				
				glUniform1f(ScaleID, scale);
				TagTextures[number].Draw();
				Meshes[MeshNames::tag].Draw(ParameterID, true);
			}
			break;
		case MeshNames::camera : //Add an axis to the camera
			//Meshes[MeshNames::axis].Draw(ParameterID);
			[[fallthrough]];
		default:
			Meshes[odata.type].Draw(ParameterID);
			break;
		}
	}

	glfwSwapBuffers(Window);

	bool IsDone = glfwGetKey(Window, GLFW_KEY_ESCAPE ) != GLFW_PRESS && glfwWindowShouldClose(Window) == 0;

	if (DoScreenCapture())
	{
		CaptureWindow(GetScreenCapturePath()/"3D"/(to_string(FrameIndex)+".jpeg"));
	}
	

	glfwMakeContextCurrent(NULL);
	FrameIndex++;
	return IsDone;
}

void BoardGL::runTest()
{
	int winwidth, winheight;
	glfwGetWindowSize(Window, &winwidth, &winheight);
	

	lastTime = glfwGetTime();

	do{
		HandleInputs();

		glm::vec3 direction = GetDirection();
		glm::vec3 right = GetRightVector();

		// Up vector : perpendicular to both direction and right
		glm::vec3 up = glm::cross(right, direction);

		// Clear the screen. It's not mentioned before Tutorial 02, but it can cause flickering, so it's there nonetheless.
		glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glUseProgram(ShaderProgram.ProgramID);



		glm::mat4 MVPmatrix = GetVPMatrix(direction, up) /** model*/;

		// Get a handle for our "MVP" uniform
		// Only during the initialisation
		GLuint MatrixID = glGetUniformLocation(ShaderProgram.ProgramID, "MVP");
		
		// Send our transformation to the currently bound shader, in the "MVP" uniform
		// This is done in the main loop since each model will have a different MVP matrix (At least for the M part)
		glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVPmatrix[0][0]);

		Meshes[MeshNames::robot].Draw();

		// Swap buffers
		glfwSwapBuffers(Window);
		glfwPollEvents();

	} // Check if the ESC key was pressed or the Window was closed
	while( glfwGetKey(Window, GLFW_KEY_ESCAPE ) != GLFW_PRESS &&
		glfwWindowShouldClose(Window) == 0 );
}