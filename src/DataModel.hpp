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

class Omni {
// It stores the equirectangular image together with
// the file name of the image
// and the image number (counter as it is pushed to the queue)
private:
	cv::Mat * p_img { };
	std::string imgName { };
	int imgNum { };
public:
	Omni() {
		p_img = new cv::Mat { };
	}
	Omni(const cv::Mat &img, int imgNum, std::string &imgName) {
		p_img = new cv::Mat { img };
		this->imgNum = imgNum;
		this->imgName = imgName;
	}
	~Omni() {
		delete p_img;
	}
	Omni(const Omni &a) {
		p_img = new cv::Mat {*(a.p_img)};
		imgName = a.imgName;
		imgNum = a.imgNum;
	}
	Omni(Omni &&a) {
		p_img = a.p_img;
		a.p_img = nullptr;
	}
	void setImgName (const std::string name) {
		imgName = name;
	}
	void setImgNum (const int number) {
		imgNum = number;
	}
	cv::Mat & getImage() {
		return *p_img;
	}
	std::string & getImgName() {
		return imgName;
	}
	int getImgNum() {
		return imgNum;
	}
	Omni & operator=(const Omni &a) {
		if (this != &a) {
			delete p_img;
			p_img = new cv::Mat { *(a.p_img) };
			imgName = a.imgName;
			imgNum = a.imgNum;
		}
		return *this;
	}
	Omni & operator=(Omni &&a) {
		if (this != &a) {
			delete p_img;
			p_img = a.p_img;
			a.p_img = nullptr;
			imgName = a.imgName;
			imgNum = a.imgNum;
		}
		return *this;
	}
	std::string idString() {
		return std::to_string(imgNum);
	}
};

//Omnidirectional image, img is composed by the two camera pictures
class OmniWithFeatures {
private:
	std::shared_ptr<Omni> omni;
	std::vector<KeyPoint> kpts; //Keypoints extracted from img.
	cv::Mat desc; // descriptors
public:
	OmniWithFeatures() {}
	OmniWithFeatures(shared_ptr<Omni> omni) :
			omni(omni) {
	}
	std::shared_ptr<Omni> & getOmni() {
		return omni;
	}
	std::vector<KeyPoint> & getKeyPoint() {
		return kpts;
	}
	cv::Mat & getDesc() {
		return desc;
	}
	void setOmni (const std::shared_ptr<Omni> &p) {
		omni = p;
	}
	void setKeyPoint(const std::vector<KeyPoint> & k) {
		kpts = k;
	}
	void setDesc(const cv::Mat &d) {
		desc = d;
	}
	std::string & getImgName() {
		return omni->getImgName();
	}
	string idString() {
		return omni->idString();
	}
};

class PairWithMatches {
public:
	vector<DMatch> matches;
	std::shared_ptr<OmniWithFeatures> imgs[2];

	PairWithMatches() {
	}
	PairWithMatches(std::shared_ptr<OmniWithFeatures> img1, std::shared_ptr<OmniWithFeatures> img2) :
			imgs( { img1, img2 }) {
	}
	;
	string idString() {
		return "(" + imgs[0]->idString() + " " + imgs[1]->idString() + ")";
	}
};

class TripletsWithMatches {
public:
	std::shared_ptr<OmniWithFeatures> imgs[3];
	vector<vector<int>> matches;

	TripletsWithMatches() {
	}
	;
	TripletsWithMatches(std::shared_ptr<OmniWithFeatures> img0, std::shared_ptr<OmniWithFeatures> img1, std::shared_ptr<OmniWithFeatures> img2) {
		this->imgs[0] = img0;
		this->imgs[1] = img1;
		this->imgs[2] = img2;
	}
	;
	string idString() {
		return "(" + imgs[0]->idString() + " " + imgs[1]->idString() + " " + imgs[2]->idString() + ")";
	}
};


typedef Vec<uint8_t, 3> RGB888;
class ModelFeature {
public:
	Matx13f position;
	RGB888 color;
	ModelFeature() {
	}
	ModelFeature(Matx13f position) :
			position(position) {
	}
	ModelFeature(Matx13f position, RGB888 color) :
			position(position), color(color) {
	}
};

class ModelKeypoint {
public:
	Matx13f position;

	ModelKeypoint() {
	}
	ModelKeypoint(Matx13f position) :
			position(position) {
	}
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
