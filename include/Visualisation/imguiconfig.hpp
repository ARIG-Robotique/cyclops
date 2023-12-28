//#include <GL/glew.h>
//#include <GLFW/glfw3.h>
#include <opencv2/core.hpp>

//#define ImTextureID GLuint

#define IM_VEC2_CLASS_EXTRA																						\
		template<class T> constexpr ImVec2(const cv::Size_<T>& f) : x(f.width), y(f.height) {}					\
		template<class T> operator cv::Size_<T>() const { return cv::Size_<T>(x,y); }							\
		template<class T> constexpr ImVec2(const cv::Point_<T>& f) : x(f.x), y(f.y) {}							\
		template<class T> operator cv::Point_<T>() const { return cv::Point_<T>(x,y); }							
