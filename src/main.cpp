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

#include "ctpl.hpp"
#include "Queue.hpp"

using namespace std;

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

class Model {
	// change to a list of indices
	int idx1 = 0;
	int idx2 = 1;
	int idx3 = 2;
	// List of points for models
public:
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


Model FusionModel (Model &m1, Model &m2) {
	std::stringstream ss { };
	ss << "=========================" << std::endl << "Fusion Model (" << m1.getListIdx1() << " - " << m1.getListIdx2() << " - " << m1.getListIdx3() << ") ("
			<< m2.getListIdx1() << " - " << m2.getListIdx2() << " - " << m2.getListIdx3() << ") " << std::endl
			<< "=========================" << std::endl;
	print(ss.str());

	Model m3 {};
	return m3;
}

void ProcPose() {

	std::vector <Model> vecm {};
	Model m;


	for (;;) {
		std::shared_ptr<TripletsImages> receivedTripletsImages { };
		receivedTripletsImages = tripletsProcQueue.wait_pop();
		std::stringstream ss {};
		ss << "=========================" << std::endl << "Received triplets images " << receivedTripletsImages->getListIdx1() << ", " << receivedTripletsImages->getListIdx2() << ", " << receivedTripletsImages->getListIdx3() << std::endl << "=========================" << std::endl;
		print (ss.str());

		Triplets t1 {receivedTripletsImages->getListIdx1(), receivedTripletsImages->getListIdx2(), receivedTripletsImages->getListIdx3()};
		Model m1 {PoseEstimation (t1)};

		m = FusionModel (m1, m);
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
