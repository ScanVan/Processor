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

#include "ctpl.hpp"
#include "PairImages.hpp"
#include "Queue.hpp"

using namespace std;

ScanVan::thread_safe_queue<ScanVan::PairImages> imgProcQueue {};
ctpl::thread_pool p(4 /* two threads in the pool */);
std::mutex mtx{};


void print (std::string st) {
	std::unique_lock<std::mutex> lck {mtx};
	std::cout << st;
}

void GeneratePairImages () {

	int i {0};
	for (;;) {
		i++;
		ScanVan::Images im1{};
		ScanVan::Images im2{};
		ScanVan::PairImages p1{ im1, im2};
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		p1.setImgNumber(i);
		imgProcQueue.push(p1);
		std::stringstream ss {};
		ss << "Send pair images " << to_string(p1.getImgNumber()) << std::endl;
		print (ss.str());
	}
}

void FeatureExtraction (int id, std::shared_ptr<ScanVan::PairImages> im1, std::shared_ptr<ScanVan::PairImages> im2) {

	std::stringstream ss {};
	ss << "Feature Extraction " << im1->getImgNumber() << " - "  << im2->getImgNumber() << std::endl;
	print (ss.str());

}

void DispatchPairImages() {

	std::vector<std::shared_ptr<ScanVan::PairImages>> v{};

	for (;;) {
		std::shared_ptr<ScanVan::PairImages> receivedPairImages { };
		receivedPairImages = imgProcQueue.wait_pop();
		std::stringstream ss {};
		ss << "Received pair images " << receivedPairImages->getImgNumber() << std::endl;
		print (ss.str());

/*
		v.push_back(receivedPairImages);
		if (v.size()==2) {
			p.push(FeatureExtraction, v[0], v[1]);
			v[0]=v[1];
			v.pop_back();
		}
*/
	}
}


int main() {


	std::thread GenPairs (GeneratePairImages);
	std::thread DispatchPairs (DispatchPairImages);

	GenPairs.join();
	DispatchPairs.join();

	return 0;

}
