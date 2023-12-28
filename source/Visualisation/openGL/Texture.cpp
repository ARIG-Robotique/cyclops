#include "Visualisation/openGL/Texture.hpp"

#include <opencv2/imgcodecs.hpp>

using namespace std;

Texture::~Texture()
{
	if (TextureID)
	{
		glDeleteTextures(1, &TextureID);
	}
}

void Texture::LoadFromFile(string path)
{
	SourceImage = cv::imread(path, cv::IMREAD_COLOR);
	valid = true;
}

void Texture::LoadFromUMat(const cv::UMat &Image)
{
	SourceImage = Image.getMat(cv::AccessFlag::ACCESS_READ | cv::AccessFlag::ACCESS_FAST);
	if (valid)
	{
		Refresh();
	}
	else
	{
		Bind();
		valid = true;
	}
	SourceImage = cv::Mat();
}

void Texture::Bind()
{
	glGenTextures(1, &TextureID);
	LastSize = cv::Size(0,0);

	Refresh();

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
}

void Texture::Refresh()
{
	glBindTexture(GL_TEXTURE_2D, TextureID);

	cv::Size NewSize(SourceImage.cols, SourceImage.rows);
	int numchannels = SourceImage.channels();
	GLenum format = numchannels == 3 ? GL_BGR : GL_R;
	GLint internal_format = numchannels == 3 ? GL_RGB : GL_R;
	if (NewSize != LastSize)
	{
		glTexImage2D(GL_TEXTURE_2D, 0, internal_format, SourceImage.cols, SourceImage.rows, 0, format, GL_UNSIGNED_BYTE, SourceImage.data);
	}
	else
	{
		glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, SourceImage.cols, SourceImage.rows, format, GL_UNSIGNED_BYTE, SourceImage.data);
	}
	LastSize = NewSize;
}

void Texture::Draw()
{
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, TextureID);
}