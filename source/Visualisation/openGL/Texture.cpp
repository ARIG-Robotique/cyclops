#include "Visualisation/openGL/Texture.hpp"

#include <opencv2/imgcodecs.hpp>
#include <iostream>

using namespace std;

#define gl_check_err() {GLenum stoerr = glGetError(); if(stoerr != GL_NO_ERROR) {cout << "@ line " << __LINE__ << " error " << gluErrorString(stoerr) <<endl; assert(0);}}

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
	glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
	gl_check_err();
	glGenTextures(1, &TextureID);
	gl_check_err();
	LastSize = cv::Size(0,0);
	//cout << "glGenTextures " << TextureID << " error " << gluErrorString(glGetError()) << endl;

	Refresh();

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	gl_check_err();
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	gl_check_err();
	
}

void Texture::Refresh()
{
	glBindTexture(GL_TEXTURE_2D, TextureID);
	gl_check_err();
	cv::Size NewSize = SourceImage.size();
	int numchannels = SourceImage.channels();
	GLenum format = numchannels == 3 ? GL_BGR : GL_R;
	GLint internal_format = numchannels == 3 ? GL_RGB : GL_R;
	//cout << "Bound to " << TextureID << " @ " << this << ", resolution " << NewSize << endl;
	if ((NewSize.width*numchannels) % 4)
	{
		cout << "Warning : Texture width (" << NewSize.width << "*" << numchannels << ") not multiple of 4, misaligned!" << endl;
	}
	
	if (NewSize != LastSize)
	{
		assert(LastSize == cv::Size());
		glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
		gl_check_err();
		glTexImage2D(GL_TEXTURE_2D, 0, internal_format, NewSize.width, NewSize.height, 0, format, GL_UNSIGNED_BYTE, SourceImage.data);
		gl_check_err();
		//cout << "glTexImage2D " << TextureID << " error " << gluErrorString(glGetError()) << endl;
	}
	else
	{

		glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, NewSize.width, NewSize.height, format, GL_UNSIGNED_BYTE, SourceImage.data);
		gl_check_err();
	}
	//cout << "Texture " << TextureID << " done updating" << endl;
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