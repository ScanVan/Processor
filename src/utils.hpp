#ifndef SRC_UTILS_HPP_
#define SRC_UTILS_HPP_

#include "Vec_Points.hpp"
#include <string>
#include <opencv4/opencv2/core/types.hpp>

class ModelViewPoint;
class ModelFeature;

using namespace std;
using namespace cv;



#define DEBUG_PTR(ptr) auto ptr##_ = ptr.get();

void print (std::string st);
void writePly(string file, vector<ModelFeature> &features);
void writePly(const std::string &file, const Vec_Points<double> &features);
vector<ModelFeature> keypointsToFeatures(vector<ModelViewPoint> *keypoints);
Matx13f cross(Matx13f a, Matx13f b);

#endif /* SRC_UTILS_HPP_ */
