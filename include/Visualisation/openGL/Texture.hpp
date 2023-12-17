#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include <GL/glew.h>
#include <glm/glm.hpp>

struct Texture
{
private:
	GLuint TextureID = 0;

	cv::Size LastSize;

public:
	cv::Mat SourceImage;

	bool valid = false;

	Texture()
	{}

	virtual ~Texture();

	void LoadFromFile(std::string path);

	void LoadFromUMat(const cv::UMat &Image);

	void Bind(); //Send the texutre to the GPU

	void Refresh(); //Update the GPU texture with the one inside here

	void Draw(); //Set the texture as the currently active texture

	GLuint GetTextureID() const
	{
		return TextureID;
	}
};
