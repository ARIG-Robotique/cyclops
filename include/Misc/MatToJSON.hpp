#pragma once

#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
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
nlohmann::json Point3ToJson(const cv::Point3_<T> &point)
{
	nlohmann::json object;
	object["x"] = point.x;
	object["y"] = point.y;
	object["z"] = point.z;
	return object;
}

template<class T>
cv::Point3_<T> JsonToPoint3(const nlohmann::json &object)
{
	cv::Point3_<T> point(object.at("x"), object.at("y"), object.at("z"));
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
nlohmann::json Affine3ToJson(const cv::Affine3<T> &pos)
{
	nlohmann::json object;
	cv::Vec<T, 3> translation = pos.translation(), rotVec;
	auto rotation = pos.rotation();
	cv::Rodrigues(rotation, rotVec);
	object["translation"] = Point3ToJson<T>(translation);
	object["rotation"] = Point3ToJson<T>(rotVec);
	return object;
}

template<class T>
cv::Affine3<T> JsonToAffine3(const nlohmann::json &object)
{
	cv::Vec<T, 3> translation = JsonToPoint3<T>(object.at("translation")), rotVec = JsonToPoint3<T>(object.at("rotation"));
	return cv::Affine3<T>(rotVec, translation);
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