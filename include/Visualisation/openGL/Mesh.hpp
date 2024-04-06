#pragma once

#include <string>

#include <vector>
#include <filesystem>
#include <GL/glew.h>
#include <glm/glm.hpp>

#include <Visualisation/openGL/Texture.hpp>

//A 3D model
struct Mesh
{
private:
	GLuint PositionBuffer;
	GLuint UVBuffer;
	GLuint ColorBuffer;

	GLuint IndexBuffer;
	bool bound = false;

public:
	std::vector<GLfloat> Positions;
	std::vector<GLfloat> UVs;
	std::vector<GLfloat> Colors;

	std::vector<unsigned int> Indices;

	Texture texture;

	virtual ~Mesh();

	bool LoadMesh(std::filesystem::path path);

	bool LoadTexture(cv::Mat Texture);
	bool LoadTexture(std::filesystem::path path);

	bool LoadFromFile(std::filesystem::path path, std::filesystem::path texturepath = "");

	void BindMesh(); //Create buffers for OpenGL

	void Draw(GLuint ParamHandle = UINT32_MAX, bool forceTexture = false); //Draw the mesh

	void Release(); //Forget bound buffers
};
