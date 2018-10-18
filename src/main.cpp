//============================================================================
// Name        : Processor.cpp
// Author      : Marcelo E. Kaihara
// Version     :
// Copyright   : 
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <thread>
#include <mutex>
#include <sstream>
#include <opencv4/opencv2/core/types.hpp>
#include <opencv4/opencv2/core.hpp>

#include "ctpl.hpp"
#include "Queue.hpp"

using namespace std;
using namespace cv;

class PairImages;
class TripletsImages;

ScanVan::thread_safe_queue<PairImages> imgProcQueue {};
ScanVan::thread_safe_queue<TripletsImages> tripletsProcQueue {};
ctpl::thread_pool p(4 /* two threads in the pool */);
std::mutex mtx{};

void print (std::string st) {
	std::unique_lock<std::mutex> lck {mtx};
	std::cout << st;
}

class PairImages {
	int imgNum = 0;
public:
	PairImages() { std::stringstream ss{}; ss << "-->Image pair constructed." << std::endl; print(ss.str()); };
	PairImages(PairImages &n) { imgNum = n.imgNum; std::stringstream ss{}; ss << "-->Copy constructor. Image pair " << imgNum << " constructed." << std::endl; print(ss.str()); };
	PairImages(PairImages &&n) { imgNum = n.imgNum; std::stringstream ss{}; ss << "-->Move constructor. Image pair " << imgNum << " constructed." << std::endl; print(ss.str()); };
	PairImages(int &n) { imgNum = n; std::stringstream ss{}; ss << "-->Image pair " << imgNum << " constructed." << std::endl; print(ss.str()); };
	void setImgNum (int n) { imgNum = n; }
	int getImgNum() const { return imgNum; }
	~PairImages() {  std::stringstream ss{}; ss << "-->Image pair " << imgNum << " destructed." << std::endl; print(ss.str()); };
	PairImages & operator= (PairImages &p) { imgNum = p.imgNum; std::stringstream ss{}; ss << "Image pair " << imgNum << " copied." << std::endl; print(ss.str()); return *this;};
};

class PairListPoints {
	int idx1 = 0;
	int idx2 = 1;
public:
	PairListPoints() { std::stringstream ss{}; ss << "-->Pair of list of points constructed." << std::endl; print(ss.str()); };
	PairListPoints(int n1, int n2) : idx1 {n1}, idx2{n2} { std::stringstream ss{}; ss << "-->Pair of list of points " << idx1 << ", " << idx2 << " constructed." << std::endl; print(ss.str()); };
	PairListPoints(PairListPoints &n) { idx1 = n.idx1; idx2 = n.idx2; std::stringstream ss{}; ss << "-->Copy constructor. Pair of list of points " << idx1 << ", " << idx2 << " constructed." << std::endl; print(ss.str()); };
	PairListPoints(PairListPoints &&n) { idx1 = n.idx1; idx2 = n.idx2; std::stringstream ss{}; ss << "-->Move constructor. Pair of list of points " << idx1 << ", " << idx2 << " constructed." << std::endl; print(ss.str()); };
	void setListIdx (int a, int b) { idx1 = a; idx2 = b; }
	int getListIdx1 () const { return idx1; }
	int getListIdx2 () const { return idx2; }
	PairListPoints & operator=(const PairListPoints &n) {idx1 = n.idx1; idx2 = n.idx2; std::stringstream ss{}; ss << "-->Assignment operator. Pair of list of points " << idx1 << ", " << idx2 << " constructed." << std::endl; print(ss.str()); return *this;};
	~PairListPoints() { std::stringstream ss{}; ss << "-->Pair of list of points " << idx1 << ", " << idx2 << " destructed." << std::endl; print(ss.str()); };
};

class Triplets {
	int idx1 = 0;
	int idx2 = 1;
	int idx3 = 2;
	// List of points
public:
	Triplets() { std::stringstream ss{}; ss << "-->Triplets constructed." << std::endl; print(ss.str()); };
	Triplets(int n1, int n2, int n3) : idx1 {n1}, idx2{n2}, idx3{n3} { std::stringstream ss{}; ss << "-->Triplets " << idx1 << ", " << idx2 << ", " << idx3 << " constructed." << std::endl; print(ss.str()); };
	Triplets(Triplets &n) { idx1 = n.idx1; idx2 = n.idx2; idx3 = n.idx3; std::stringstream ss{}; ss << "-->Copy constructor. Triplets " << idx1 << ", " << idx2 << ", " << idx3 << " constructed." << std::endl; print(ss.str()); };
	Triplets(Triplets &&n) { idx1 = n.idx1; idx2 = n.idx2; idx3 = n.idx3; std::stringstream ss{}; ss << "-->Move constructor. Triplets " << idx1 << ", " << idx2 << ", " << idx3 << " constructed." << std::endl; print(ss.str()); };
	void setListIdx (int a, int b, int c) { idx1 = a; idx2 = b; idx3 = c;}
	int getListIdx1 () const { return idx1; }
	int getListIdx2 () const { return idx2; }
	int getListIdx3 () const { return idx3; }
	Triplets & operator= (Triplets &n) { idx1 = n.idx1; idx2 = n.idx2; idx3 = n.idx3; std::stringstream ss{}; ss << "-->Assignment operator. Triplets " << idx1 << ", " << idx2 << ", " << idx3 << " constructed." << std::endl; print(ss.str()); return *this;};
	~Triplets() { std::stringstream ss{}; ss << "-->Triplets " << idx1 << ", " << idx2 << ", " << idx3 << " destructed." << std::endl; print(ss.str()); };
};

class TripletsImages {
	int idx1 = 0;
	int idx2 = 1;
	int idx3 = 2;
	std::shared_ptr<PairImages> img1 {};
	std::shared_ptr<PairImages> img2 {};
	std::shared_ptr<PairImages> img3 {};
	// List of points
public:
	TripletsImages() { std::stringstream ss{}; ss << "-->TripletsImages constructed." << std::endl; print(ss.str()); };
	TripletsImages(int n1, int n2, int n3, std::shared_ptr<PairImages> im1, std::shared_ptr<PairImages> im2, std::shared_ptr<PairImages> im3) :
			idx1 { n1 }, idx2 { n2 }, idx3 { n3 }, img1 {im1}, img2 {im2}, img3 {im3}  {
		std::stringstream ss { };
		ss << "-->TripletsImages " << idx1 << ", " << idx2 << ", " << idx3 << " constructed." << std::endl;
		print(ss.str());
	}
	TripletsImages(TripletsImages &n) {
		idx1 = n.idx1;
		idx2 = n.idx2;
		idx3 = n.idx3;
		img1 = n.img1;
		img2 = n.img2;
		img3 = n.img3;
		std::stringstream ss { };
		ss << "-->Copy constructor. TripletsImages " << idx1 << ", " << idx2 << ", " << idx3 << " constructed." << std::endl;
		print(ss.str());
	}
	TripletsImages(TripletsImages &&n) {
		idx1 = n.idx1;
		idx2 = n.idx2;
		idx3 = n.idx3;
		img1 = n.img1;
		img2 = n.img2;
		img3 = n.img3;
		std::stringstream ss { };
		ss << "-->Move constructor. TripletsImages " << idx1 << ", " << idx2 << ", " << idx3 << " constructed." << std::endl;
		print(ss.str());
	}
	void setListIdx (int a, int b, int c) { idx1 = a; idx2 = b; idx3 = c;}
	void setSharedPtr(std::shared_ptr<PairImages> a, std::shared_ptr<PairImages> b, std::shared_ptr<PairImages> c) {
		img1 = a;
		img2 = b;
		img3 = c;
	}
	int getListIdx1 () const { return idx1; }
	int getListIdx2 () const { return idx2; }
	int getListIdx3 () const { return idx3; }
	TripletsImages & operator=(TripletsImages &n) {
		idx1 = n.idx1;
		idx2 = n.idx2;
		idx3 = n.idx3;
		img1 = n.img1;
		img2 = n.img2;
		img3 = n.img3;
		std::stringstream ss { };
		ss << "-->Assignment operator. TripletsImages " << idx1 << ", " << idx2 << ", " << idx3 << " constructed." << std::endl;
		print(ss.str());
		return *this;
	}
	~TripletsImages() {
		std::stringstream ss { };
		ss << "-->TripletsImages " << idx1 << ", " << idx2 << ", " << idx3 << " destructed." << std::endl;
		print(ss.str());
	}

};



typedef Vec<uint8_t, 3> RGB888;

class ModelFeature{
public:
	Point3f position;
	RGB888 color;
};

//Represent
class ModelKeypoint{
public:
	Point3f position;
	//...
};



class Model {
	// change to a list of indices
	int idx1 = 0;
	int idx2 = 1;
	int idx3 = 2;
	// List of points for models

public:
	vector<ModelKeypoint> keypoints;
	vector<ModelFeature> features;


	Model() {
		std::stringstream ss { };
		ss << "-->Model constructed." << std::endl;
		print(ss.str());
	}
	Model(int n1, int n2, int n3) :
			idx1 { n1 }, idx2 { n2 }, idx3 { n3 } {
		std::stringstream ss { };
		ss << "-->Model " << idx1 << ", " << idx2 << ", " << idx3 << " constructed." << std::endl;
		print(ss.str());
	}
	Model(Model &n) {
		idx1 = n.idx1;
		idx2 = n.idx2;
		idx3 = n.idx3;
		std::stringstream ss { };
		ss << "-->Copy constructor. Model " << idx1 << ", " << idx2 << ", " << idx3 << " constructed." << std::endl;
		print(ss.str());
	}
	Model(Model &&n) {
		idx1 = n.idx1;
		idx2 = n.idx2;
		idx3 = n.idx3;
		std::stringstream ss { };
		ss << "-->Move constructor. Model " << idx1 << ", " << idx2 << ", " << idx3 << " constructed." << std::endl;
		print(ss.str());
	}

	void setListIdx(int a, int b, int c) {
		idx1 = a;
		idx2 = b;
		idx3 = c;
	}
	int getListIdx1() const {
		return idx1;
	}
	int getListIdx2() const {
		return idx2;
	}
	int getListIdx3() const {
		return idx3;
	}
	Model & operator=(Model &n) {
		idx1 = n.idx1;
		idx2 = n.idx2;
		idx3 = n.idx3;
		std::stringstream ss { };
		ss << "-->Assignment operator. Model " << idx1 << ", " << idx2 << ", " << idx3 << " constructed." << std::endl;
		print(ss.str());
		return *this;
	}
	Model & operator=(Model &&n) {
		idx1 = n.idx1;
		idx2 = n.idx2;
		idx3 = n.idx3;
		std::stringstream ss { };
		ss << "-->Move assignment operator. Model " << idx1 << ", " << idx2 << ", " << idx3 << " constructed." << std::endl;
		print(ss.str());
		return *this;
	}

	~Model() {
		std::stringstream ss { };
		ss << "-->Model " << idx1 << ", " << idx2 << ", " << idx3 << " destructed." << std::endl;
		print(ss.str());
	}
};

//=========================================================================================================

void GeneratePairImages () {

	int i {0};

	for (;;) {
		i++;
		PairImages p1{i};
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		imgProcQueue.push(p1);
		std::stringstream ss {};
		ss << "=========================" << std::endl << "Send pair images " << p1.getImgNum() << std::endl << "=========================" << std::endl;
		print (ss.str());
	}
}

PairListPoints FeatureExtraction (std::shared_ptr<PairImages> im1, std::shared_ptr<PairImages> im2) {

	std::stringstream ss {};
	ss << "=========================" << std::endl << "Feature Extraction " << im1->getImgNum() << " - "  << im2->getImgNum() << std::endl << "=========================" << std::endl;
	print (ss.str());

	PairListPoints p1 { im1->getImgNum(), im2->getImgNum() };
	return p1;
}

Triplets CommonPointsComputation (PairListPoints &p1, PairListPoints &p2) {

	std::stringstream ss {};
	ss << "=========================" << std::endl << "Common Points Computation (" << p1.getListIdx1() << " - "  << p1.getListIdx2() << ") (" << p2.getListIdx1() << " - " << p2.getListIdx2() << ")" <<  std::endl << "=========================" << std::endl;
	print (ss.str());

	if (p1.getListIdx2() != p2.getListIdx1()) {
		throw std::runtime_error ("Error in indexes in CommonPointComputation");
	}

	Triplets t1 { p1.getListIdx1(), p1.getListIdx2(), p2.getListIdx2()};

	return t1;
}

void ProcFeatures() {

	std::vector<std::shared_ptr<PairImages>> v{};
	std::vector<std::shared_ptr<PairImages>> ims{};
	std::vector<PairListPoints> lp{};

	for (;;) {
		std::shared_ptr<PairImages> receivedPairImages { };
		receivedPairImages = imgProcQueue.wait_pop();
		std::stringstream ss {};
		ss << "=========================" << std::endl << "Received pair images " << receivedPairImages->getImgNum() << std::endl << "=========================" << std::endl;
		print (ss.str());

		v.push_back(receivedPairImages);
		ims.push_back(receivedPairImages);

		if (v.size() == 2) {
			lp.push_back(FeatureExtraction (v[0], v[1]));
			v[0]=v[1];
			v.pop_back();
		}

		if (lp.size() == 2) {
			Triplets p1 {CommonPointsComputation (lp[0], lp[1])};
			lp[0] = lp[1];
			lp.pop_back();
			TripletsImages ti { p1.getListIdx1(), p1.getListIdx2(), p1.getListIdx3(), ims[0], ims[1], ims[2] };
			for (auto it = ims.begin(); it!= ims.end()-1; ++it) {
				*it = *(it+1);
			}
			ims.pop_back();
			tripletsProcQueue.push(ti);
		}
	}
}

Model PoseEstimation (Triplets &t1) {
	std::stringstream ss { };
	ss << "=========================" << std::endl << "Pose Estimation with triplets" << t1.getListIdx1() << " - " << t1.getListIdx2() << " - " << t1.getListIdx3() << std::endl
			<< "=========================" << std::endl;
	print(ss.str());

	Model m1 { t1.getListIdx1(), t1.getListIdx2(), t1.getListIdx3() };
	return m1;
}


void FusionModel (Model *m1, Model *m2) {
	std::stringstream ss { };
	ss << "=========================" << std::endl << "Fusion Model (" << m1->getListIdx1() << " - " << m1->getListIdx2() << " - " << m1->getListIdx3() << ")"
			<< std::endl
			<< "=========================" << std::endl;
	print(ss.str());

	assert(m1->keypoints.size() >= 3);
	assert(m2->keypoints.size() == 3);
	auto m1CapturesSize = m1->keypoints.size();
	auto m1C1 = m1->keypoints[m1CapturesSize-2].position;
	auto m1C2 = m1->keypoints[m1CapturesSize-1].position;
	auto m2C1 = m2->keypoints[0].position;
	auto m2C2 = m2->keypoints[1].position;
	auto m2C3 = m2->keypoints[2].position;
	auto scaleFactor = norm(m1C2-m1C1)/norm(m2C2-m2C1);
	auto t1 =(m2C1-m1C1)/norm(m2C1-m1C1);
	auto t2 =(m2C2-m2C1)/norm(m2C2-m2C1);
	auto v = t1.cross(t2);
	auto c = t1.dot(t2);
	auto w = Matx33f(0.0,-v.z,v.y,v.z,0.0,-v.x,-v.y,v.x,0.0);
	auto rotation = Matx33f().eye() + (w.dot(w)/(1+c)) * w;
	auto translation = m1C1-m1C2;

	for(auto source : m2->features){
		auto feature = ModelFeature();
		feature.color = source.color;
		feature.position = translation + scaleFactor * (rotation * source.position);
		m1->features.push_back(feature);
	}
	ModelKeypoint m1C3;
	m1C3.position = translation + scaleFactor * (rotation * m2C3);
	m1->keypoints.push_back(m1C3);

//# model = [positions,sv_scene] output form the main iterative algorithm
//pos_1=model_1[0]
//scene_1=model_1[1]
//pos_2=model_2[0]
//scene_2=model_2[1]
//#position used for aligment
//pos_a_1=np.asarray(pos_1[-2])
//pos_b_1=np.asarray(pos_1[-1])
//pos_a_2=np.asarray(pos_2[0])
//pos_b_2=np.asarray(pos_2[1])
//#position to add (after aligment)
//pos_c=np.asarray(pos_2[2])
//#scale factor calculus
//scale_factor=np.linalg.norm(pos_b_1-pos_a_1)/np.linalg.norm(pos_b_2-pos_a_2)
//#rotation calculus
//t1=(pos_b_1-pos_a_1)/np.linalg.norm(pos_b_1-pos_a_1)
//t2=(pos_b_2-pos_a_2)/np.linalg.norm(pos_b_2-pos_a_2)
//v=np.cross(t1,t2)
//c=np.dot(t1,t2)
//w=np.matrix([[0.0,-v[2],v[1]],[v[2],0.0,-v[0]],[-v[1],v[0],0.0]])
//rotation=np.identity(3)+w+np.dot(w,w)/(1+c)
//#translation calculus
//translation=pos_a_1-pos_a_2
//#new scene with point form both models
//scene=[]
//for i in range(len(scene_1)):
//    point=tuple(np.squeeze(np.asarray(scene_1[i])))
//    scene.append(point)
//for i in range(len(scene_2)):
//    point=tuple(np.squeeze(np.asarray(translation+scale_factor*np.dot(scene_2[i],rotation))))
//    scene.append(point)
//# new positions array
//positions=pos_1+[np.squeeze(np.asarray(translation+scale_factor*np.dot(pos_c,rotation)))]
//model=[positions,scene]

}

void ProcPose() {

	std::vector <Model> vecm {};
	Model m {};


	for (;;) {
		std::shared_ptr<TripletsImages> receivedTripletsImages { };
		receivedTripletsImages = tripletsProcQueue.wait_pop();
		std::stringstream ss {};
		ss << "=========================" << std::endl << "Received triplets images " << receivedTripletsImages->getListIdx1() << ", " << receivedTripletsImages->getListIdx2() << ", " << receivedTripletsImages->getListIdx3() << std::endl << "=========================" << std::endl;
		print (ss.str());

		Triplets t1 {receivedTripletsImages->getListIdx1(), receivedTripletsImages->getListIdx2(), receivedTripletsImages->getListIdx3()};
		Model m1 {PoseEstimation (t1)};

		FusionModel (&m1, &m);
	}
}


int main() {


	std::thread GenPairs (GeneratePairImages);
	std::thread ProcessFeatureExtraction (ProcFeatures);
	std::thread ProcessPoseEstimation (ProcPose);

	GenPairs.join();
	ProcessFeatureExtraction.join();
	ProcessPoseEstimation.join();

	return 0;

}
