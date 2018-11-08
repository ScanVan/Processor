#ifndef SRC_DATAMODEL_HPP_
#define SRC_DATAMODEL_HPP_

#include <iostream>
#include <thread>
#include <mutex>
#include <sstream>
#include <iostream>
#include <fstream>
#include <iterator>
#include <vector>
#include <opencv4/opencv2/core/types.hpp>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/features2d.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <math.h>

#include "utils.hpp"

using namespace cv;
using namespace std;

class Omni{
public:
	Mat img;
	int imgNum = 0;

	Omni(int imgNum) : imgNum(imgNum){}

	string idString() { return to_string(imgNum); }
};

//Omnidirectional image, img is composed by the two camera pictures
class OmniWithFeatures {
public:
	shared_ptr<Omni> omni;
    vector<KeyPoint> kpts; //Keypoints extracted from img.
    Mat desc;

	OmniWithFeatures(shared_ptr<Omni> omni) : omni(omni) {};
	string idString() { return omni->idString(); }
};

class PairWithMatches {
public:
    vector<DMatch> matches;
    std::shared_ptr<OmniWithFeatures> imgs[2];

	PairWithMatches() {}
	PairWithMatches(std::shared_ptr<OmniWithFeatures> img1, std::shared_ptr<OmniWithFeatures> img2) : imgs({img1,img2}) {};
	string idString() { return "(" + imgs[0]->idString() + " " + imgs[1]->idString() + ")" ; }
};

class TripletsWithMatches {
public:
	std::shared_ptr<OmniWithFeatures> imgs[3];
	vector<vector<int>> matches;


	TripletsWithMatches() {};
	TripletsWithMatches(std::shared_ptr<OmniWithFeatures> img0, std::shared_ptr<OmniWithFeatures> img1, std::shared_ptr<OmniWithFeatures> img2)  {this->imgs[0] = img0;this->imgs[1] = img1;this->imgs[2] = img2; };
	string idString() { return "(" + imgs[0]->idString() + " " + imgs[1]->idString() + imgs[2]->idString() + ")" ; }
};


typedef Vec<uint8_t, 3> RGB888;
class ModelFeature{
public:
	Matx13f position;
	RGB888 color;
	ModelFeature(){}
	ModelFeature(Matx13f position) : position(position){}
	ModelFeature(Matx13f position, RGB888 color) : position(position), color(color){}
};


class ModelKeypoint{
public:
	Matx13f position;

	ModelKeypoint(){}
	ModelKeypoint(Matx13f position) : position(position){}
};


class Model {
public:
	vector<ModelKeypoint> keypoints;
	vector<ModelFeature> features;


	Model() {
		std::stringstream ss { };
		ss << "-->Model constructed." << std::endl;
		print(ss.str());
	}
};

#endif /* SRC_DATAMODEL_HPP_ */
