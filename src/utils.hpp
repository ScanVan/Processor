#ifndef SRC_UTILS_HPP_
#define SRC_UTILS_HPP_

#include <string>
#include <opencv4/opencv2/core/types.hpp>
#include "Vec_Points.hpp"
#include "DataModel.hpp"
#include "config.hpp"

#define DEBUG_PTR(ptr) auto ptr##_ = ptr.get();

void print (std::string st);
std::vector<ModelFeature> keypointsToFeatures(std::vector<ModelViewPoint> *keypoints);
cv::Matx13f cross(cv::Matx13f a, cv::Matx13f b);

#endif /* SRC_UTILS_HPP_ */
