#include "Visualisation/openGL/Texture.hpp"

#include <opencv2/imgcodecs.hpp>
#include <iostream>

using namespace std;

Texture::~Texture()
{
	if (TextureID)
	{
		//cout << "Deleting texture " << TextureID << endl;
		glDeleteTextures(1, &TextureID);
	}
}

void Texture::LoadFromFile(std::filesystem::path path)
{
	SourceImage = cv::imread(path, cv::IMREAD_COLOR);
	valid = true;
}

void Texture::LoadFromMat(const cv::Mat &Image)
{
	SourceImage = Image;
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
	
	//cout << "Created texture " << TextureID << endl;
}

void Texture::Refresh()
{
	glBindTexture(GL_TEXTURE_2D, TextureID);

	cv::Size NewSize = SourceImage.size();
	int numchannels = SourceImage.channels();
	GLenum format = numchannels == 3 ? GL_BGR : GL_R;
	GLint internal_format = numchannels == 3 ? GL_RGB : GL_R;
	if ((NewSize.width*numchannels) % 4)
	{
		cout << "Warning : Texture width (" << NewSize.width << "*" << numchannels << ") not multiple of 4, misaligned!" << endl;
	}
	
	if (NewSize != LastSize)
	{
		glTexImage2D(GL_TEXTURE_2D, 0, internal_format, NewSize.width, NewSize.height, 0, format, GL_UNSIGNED_BYTE, SourceImage.data);
	}
	else
	{
		glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, NewSize.width, NewSize.height, format, GL_UNSIGNED_BYTE, SourceImage.data);
	}
	LastSize = NewSize;
}

void Texture::Draw()
{
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, TextureID);
}

void Texture::Release()
{
	TextureID = 0;
}