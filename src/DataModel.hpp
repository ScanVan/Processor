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

//#include "utils.hpp"

void print (std::string st);

class Equirectangular {
// It stores the equirectangular image together with
// the file name of the image
// and the image number (counter as it is pushed to the queue)
private:
	cv::Mat * p_img { };
	std::string imgName { };
	int imgNum { };
public:
	Equirectangular() {
		p_img = new cv::Mat { };
	}
	Equirectangular(const cv::Mat &img, int imgNum, std::string &imgName) {
		p_img = new cv::Mat { img };
		this->imgNum = imgNum;
		this->imgName = imgName.substr(0, imgName.find_first_of("."));
	}
	~Equirectangular() {
		delete p_img;
	}
	Equirectangular(const Equirectangular &a) {
		p_img = new cv::Mat {*(a.p_img)};
		imgName = a.imgName;
		imgNum = a.imgNum;
	}
	Equirectangular(Equirectangular &&a) {
		p_img = a.p_img;
		a.p_img = nullptr;
	}
	void setImgName (const std::string name) {
		imgName = name.substr(0, name.find_first_of("."));
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
	Equirectangular & operator=(const Equirectangular &a) {
		if (this != &a) {
			delete p_img;
			p_img = new cv::Mat { *(a.p_img) };
			imgName = a.imgName;
			imgNum = a.imgNum;
		}
		return *this;
	}
	Equirectangular & operator=(Equirectangular &&a) {
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
class EquirectangularWithFeatures {
private:
	std::shared_ptr<Equirectangular> equi;
	std::vector<cv::KeyPoint> kpts; //Keypoints extracted from img.
	cv::Mat desc; // descriptors
public:
	EquirectangularWithFeatures() {}
	EquirectangularWithFeatures(std::shared_ptr<Equirectangular> sptr_equi) :
			equi(sptr_equi) {
	}
	std::shared_ptr<Equirectangular> & getOmni() {
		return equi;
	}
	std::vector<cv::KeyPoint> & getKeyPoints() {
		return kpts;
	}
	cv::Mat & getDesc() {
		return desc;
	}
	void setOmni (const std::shared_ptr<Equirectangular> &p) {
		equi = p;
	}
	void setKeyPoint(const std::vector<cv::KeyPoint> & k) {
		kpts = k;
	}
	void setDesc(const cv::Mat &d) {
		desc = d;
	}
	std::string & getImgName() {
		return equi->getImgName();
	}
	std::string idString() {
		return equi->idString();
	}
};

class PairWithMatches {
private:
	std::vector<cv::DMatch> matches;
	std::shared_ptr<EquirectangularWithFeatures> imgs[2];
public:
	PairWithMatches() {
	}
	PairWithMatches(std::shared_ptr<EquirectangularWithFeatures> img1, std::shared_ptr<EquirectangularWithFeatures> img2) :
			imgs( { img1, img2 }) {
	}
	void setMatches (const std::vector<cv::DMatch> &m) {
		matches = m;
	}
	std::vector<cv::DMatch> & getMatches () {
		return matches;
	}
	void setImages (const std::shared_ptr<EquirectangularWithFeatures> &im1, const std::shared_ptr<EquirectangularWithFeatures> &im2) {
		imgs[0] = im1;
		imgs[1] = im2;
	}
	std::shared_ptr<EquirectangularWithFeatures> & getImage1 () {
		return imgs[0];
	}
	std::shared_ptr<EquirectangularWithFeatures> & getImage2() {
		return imgs[1];
	}
	std::vector<cv::KeyPoint> & getKeyPoints1() {
		return imgs[0]->getKeyPoints();
	}
	std::vector<cv::KeyPoint> & getKeyPoints2() {
		return imgs[1]->getKeyPoints();
	}
	std::string getPairImageName() {
		return imgs[0]->getImgName() + "_" + imgs[1]->getImgName();
	}
	int getImageNumber1() {
		return imgs[0]->getOmni()->getImgNum();
	}
	int getImageNumber2() {
		return imgs[1]->getOmni()->getImgNum();
	}
	std::string idString() {
		return "(" + imgs[0]->idString() + " " + imgs[1]->idString() + ")";
	}
};

class TripletsWithMatches {
private:
	std::shared_ptr<EquirectangularWithFeatures> imgs[3];
	std::vector<std::vector<int>> matches {};
	std::vector<int> freqMatches1 {};
	std::vector<int> freqMatches2 {};
	std::vector<int> freqMatches3 {};
public:

	TripletsWithMatches() {
	}
	TripletsWithMatches(std::shared_ptr<EquirectangularWithFeatures> img0, std::shared_ptr<EquirectangularWithFeatures> img1, std::shared_ptr<EquirectangularWithFeatures> img2) {
		this->imgs[0] = img0;
		this->imgs[1] = img1;
		this->imgs[2] = img2;
	}
	std::vector<std::vector<int>> & getMatchVector() {
		return matches;
	}
	void setFrequencyMatches1(const std::vector<int> &f) {
		freqMatches1 = f;
	}
	void setFrequencyMatches2(const std::vector<int> &f) {
		freqMatches2 = f;
	}
	void setFrequencyMatches3(const std::vector<int> &f) {
		freqMatches3 = f;
	}
	std::vector<int> & getFrequencyMatches1() {
		return freqMatches1;
	}
	std::vector<int> & getFrequencyMatches2() {
		return freqMatches2;
	}
	std::vector<int> & getFrequencyMatches3() {
		return freqMatches3;
	}
	std::string getTripletImageName() {
		return imgs[0]->getImgName() + "_" + imgs[1]->getImgName() + "_" + imgs[2]->getImgName();
	}
	std::shared_ptr<EquirectangularWithFeatures> * getImage() {
		return imgs;
	}
	std::string getImageName1() {
		return imgs[0]->getOmni()->getImgName();
	}
	std::string getImageName2() {
		return imgs[1]->getOmni()->getImgName();
	}
	std::string getImageName3() {
		return imgs[2]->getOmni()->getImgName();
	}
	int getImageNumber1() {
		return imgs[0]->getOmni()->getImgNum();
	}
	int getImageNumber2() {
		return imgs[1]->getOmni()->getImgNum();
	}
	int getImageNumber3() {
		return imgs[2]->getOmni()->getImgNum();
	}
	std::string idString() {
		return "(" + imgs[0]->idString() + " " + imgs[1]->idString() + " " + imgs[2]->idString() + ")";
	}
};

typedef cv::Vec<uint8_t, 3> RGB888;

class ModelFeature {
public:
	cv::Matx13d position;
	RGB888 color;
	ModelFeature() {
	}
	ModelFeature(cv::Matx13d position) :
			position(position) {
	}
	ModelFeature(cv::Matx13d position, RGB888 color) :
			position(position), color(color) {
	}
};

class ModelViewPoint {
public:
	cv::Matx13d position;
	cv::Matx33d rotationRelative; // relative rotation with respect to the previous camera position
	cv::Matx33d rotationAbsolute; // absolute rotation with respect to the first image

	ModelViewPoint() {
	}
	ModelViewPoint(cv::Matx13d position) :
			position(position) {
	}
	ModelViewPoint(cv::Matx13d position, cv::Matx33d rotationRelative) :
			position(position), rotationRelative(rotationRelative) {
	}
	ModelViewPoint(cv::Matx13d position, cv::Matx33d rotationRelative, cv::Matx33d rotationAbsolute) :
			position(position), rotationRelative(rotationRelative), rotationAbsolute(rotationAbsolute) {
	}

};

class Model {
public:
	std::vector<ModelViewPoint> viewPoints;
	std::vector<ModelFeature> features;
	std::vector<std::string> imgNames;

	Model() {
		std::stringstream ss { };
		ss << "-->Model constructed." << std::endl;
		print(ss.str());
	}
};

#endif /* SRC_DATAMODEL_HPP_ */
