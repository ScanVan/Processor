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

ScanVan::thread_safe_queue<PairImages> imgProcQueue {};
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

void FeatureExtraction (std::shared_ptr<PairImages> im1, std::shared_ptr<PairImages> im2) {

	std::stringstream ss {};
	ss << "=========================" << std::endl << "Feature Extraction " << im1->getImgNum() << " - "  << im2->getImgNum() << std::endl << "=========================" << std::endl;
	print (ss.str());

}

void DispatchPairImages() {

	std::vector<std::shared_ptr<PairImages>> v{};

	for (;;) {
		std::shared_ptr<PairImages> receivedPairImages { };
		receivedPairImages = imgProcQueue.wait_pop();
		std::stringstream ss {};
		ss << "=========================" << std::endl << "Received pair images " << receivedPairImages->getImgNum() << std::endl << "=========================" << std::endl;
		print (ss.str());


		v.push_back(receivedPairImages);
		if (v.size()==2) {
			FeatureExtraction (v[0], v[1]);
			v[0]=v[1];
			v.pop_back();
		}

	}
}


int main() {


	std::thread GenPairs (GeneratePairImages);
	std::thread DispatchPairs (DispatchPairImages);

	GenPairs.join();
	DispatchPairs.join();

	return 0;

}
