#pragma once

#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <cassert>
#include <vector>


template<class T>
nlohmann::json SizeToJson(const cv::Size_<T> &size)
{
	nlohmann::json object;
	object["width"] = size.width;
	object["height"] = size.height;
	return object;
}

template<class T>
cv::Point_<T> JsonToSize(const nlohmann::json &object)
{
	cv::Size_<T> size(object.at("width"), object.at("height"));
	return size;
}



template<class T>
nlohmann::json PointToJson(const cv::Point_<T> &point)
{
	nlohmann::json object;
	object["x"] = point.x;
	object["y"] = point.y;
	return object;
}

template<class T>
cv::Point_<T> JsonToPoint(const nlohmann::json &object)
{
	cv::Point_<T> point(object.at("x"), object.at("y"));
	return point;
}



template<class T>
nlohmann::json RectToJson(const cv::Rect_<T> &rect)
{
	nlohmann::json object;
	object["tl"] = PointToJson<T>(rect.tl());
	object["br"] = PointToJson<T>(rect.br());
	return object;
}

template<class T>
cv::Rect_<T> JsonToRect(const nlohmann::json &object)
{
	auto tl = JsonToPoint<T>(object.at("tl"));
	auto br = JsonToPoint<T>(object.at("br"));
	cv::Rect_<T> rect(tl, br);
	return rect;
}



template<class T>
nlohmann::json MatrixToJson(const cv::Mat &matrix)
{
	nlohmann::json object;
	for (int i = 0; i < matrix.dims; i++)
	{
		object["dimensions"][i] = matrix.size[i];
	}
	
	object["type"] = matrix.type();
	auto &data = object["data"] = nlohmann::json::array();
	for (auto i = matrix.begin<T>(); i < matrix.end<T>(); i++)
	{
		data.push_back(*i);
	}
	
	return object;
}

template<class T>
cv::Mat JsonToMatrix(const nlohmann::json &object)
{
	std::vector<int> size;
	for (auto &i : object.at("dimensions"))
	{
		size.push_back(i);
	}
	int type = object.at("type");
	auto &data = object.at("data");
	cv::Mat matrix(size, type);
	int index = 0;
	for (auto i = matrix.begin<T>(); i < matrix.end<T>(); i++)
	{
		*i = data[index++];
	}
	return matrix;
}