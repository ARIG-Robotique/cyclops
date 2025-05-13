#pragma once

#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <cassert>
#include <vector>

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