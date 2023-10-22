#include "Visualisation/openGL/Texture.hpp"

#include <opencv2/imgcodecs.hpp>

using namespace std;

void Texture::LoadFromFile(string path)
{
	Texture = cv::imread(path, cv::IMREAD_COLOR);
	valid = true;
}

void Texture::Bind()
{
	glGenTextures(1, &TextureID);
	LastSize = cv::Size(0,0);

	Refresh();

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
}

void Texture::Refresh()
{
	glBindTexture(GL_TEXTURE_2D, TextureID);

	cv::Size NewSize(Texture.cols, Texture.rows);
	if (NewSize != LastSize)
	{
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, Texture.cols, Texture.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, Texture.data);
	}
	else
	{
		glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, Texture.cols, Texture.rows, GL_BGR, GL_UNSIGNED_BYTE, Texture.data);
	}
	LastSize = NewSize;
}

void Texture::Draw()
{
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, TextureID);
}