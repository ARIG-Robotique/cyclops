//#include <GL/glew.h>
//#include <GLFW/glfw3.h>
#include <opencv2/core.hpp>

//#define ImTextureID GLuint

#define IM_VEC2_CLASS_EXTRA																	\
		constexpr ImVec2(const cv::Size& f) : x(f.width), y(f.height) {}					\
		operator cv::Size() const { return cv::Size(x,y); }									\
		constexpr ImVec2(const cv::Point2i& f) : x(f.x), y(f.y) {}					\
		operator cv::Point2i() const { return cv::Point2i(x,y); }
