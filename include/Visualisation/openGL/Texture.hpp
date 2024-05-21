#pragma once

#include <string>
#include <vector>
#include <filesystem>
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
		:SourceImage()
	{}

	//Must be deleted in the context that created it
	virtual ~Texture();

	void LoadFromFile(std::filesystem::path path);

	void LoadFromMat(const cv::Mat &Image);

	void LoadFromUMat(const cv::UMat &Image);

	void Bind(); //Send the texture to the GPU

	void Refresh(); //Update the GPU texture with the one inside here

	void Draw(); //Set the texture as the currently active texture

	void Release(); //Forget that we were bound to a texture

	GLuint GetTextureID() const
	{
		return TextureID;
	}
};
