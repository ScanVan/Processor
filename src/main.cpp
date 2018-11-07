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

#include "ctpl.hpp"
#include "Queue.hpp"
#include "Estimation.hpp"

#include "gms_matcher.h"

using namespace std;
using namespace cv;


//#define USE_ORB_FEATURE
#define USE_AKAZE_FEATURE

//#define USE_KNN_MATCHER
#define USE_GMS_FILTER

//#define DISPLAY_TRIPLET_MATCHES

class Omni;
class TripletsWithMatches;

ScanVan::thread_safe_queue<Omni> imgProcQueue {};
ScanVan::thread_safe_queue<TripletsWithMatches> tripletsProcQueue {};
ctpl::thread_pool p(4 /* two threads in the pool */);
std::mutex mtx{};

void print (std::string st) {
	std::unique_lock<std::mutex> lck {mtx};
	std::cout << st;
}

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


void writePly(string file, vector<ModelFeature> features){
	ofstream s;
	s.open (file);

	s << "ply" << endl;
	s << "format ascii 1.0 " << endl;
	s << "element vertex " << features.size() << endl;
	s << "property float32 x " << endl;
	s << "property float32 y " << endl;
	s << "property float32 z " << endl;
	s << "property uchar red" << endl;
	s << "property uchar green " << endl;
	s << "property uchar blue " << endl;
	s << "element face 0 " << endl;
	s << "property list uint8 int32 vertex_index { vertex_indices is a list of ints }" << endl;
	s << "end_header " << endl;

	for(auto f : features){
		s << f.position(0) << " " << f.position(1) << " " << f.position(2) << " " <<  (uint16_t)f.color(0) << " " << (uint16_t)f.color(1) << " " << (uint16_t)f.color(2) << endl;
	}

	s.close();
}


vector<ModelFeature> keypointsToFeatures(vector<ModelKeypoint> *keypoints){
	vector<ModelFeature> ret;
	for(k : *keypoints) ret.push_back(ModelFeature(k.position, RGB888(0,255,0)));
	return ret;
}



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

//=========================================================================================================

void generatePairImages () {
	const int staticImagesCount = 6;
	Mat staticImages[staticImagesCount];
	for(int i = 0;i < staticImagesCount;i++){
		std::stringstream ss {};
		ss << "resources/0_" << (i+1) << ".bmp";
		staticImages[i] = imread(ss.str(), IMREAD_UNCHANGED);
		cout << ss.str() << endl;
	}

	int i {0};

	for (;;) {
		shared_ptr<Omni> p1(new Omni{i});
		int staticImageId = i%(staticImagesCount*2-1);
		if(staticImageId >= staticImagesCount) staticImageId = staticImagesCount*2-1 - staticImageId;
		p1->img = staticImages[staticImageId];
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		imgProcQueue.push(p1);
		std::stringstream ss {};
		ss << "=========================" << std::endl << "Send pair images " << p1->idString() << std::endl << "=========================" << std::endl;
		print (ss.str());
		i++;
	}
}


#define DEBUG_PTR(ptr) auto ptr##_ = ptr.get();


shared_ptr<OmniWithFeatures> extractFeatures(shared_ptr<Omni> omni, shared_ptr<Mat> mask){
	DEBUG_PTR(omni);
	shared_ptr<OmniWithFeatures> featured(new OmniWithFeatures(omni));

#ifdef USE_ORB_FEATURE
	Ptr<ORB> orb = ORB::create(100000);
	orb->setFastThreshold(0);
	orb->detectAndCompute(omni->img, *mask, featured->kpts, featured->desc);
#endif

#ifdef USE_AKAZE_FEATURE
    Ptr<AKAZE> akaze = AKAZE::create(
		AKAZE::DESCRIPTOR_MLDB,
		0,  3,
		0.0001f,  4,
		4, KAZE::DIFF_PM_G2
	);
    akaze->detectAndCompute(omni->img, *mask, featured->kpts, featured->desc);
#endif
	return featured;
}


std::shared_ptr<PairWithMatches> omniMatching (std::shared_ptr<OmniWithFeatures> im1, std::shared_ptr<OmniWithFeatures> im2) {
	std::stringstream ss {};
	ss << "=========================" << std::endl << "Feature Extraction (" << im1->idString() << " "  << im2->idString() << ")" << std::endl << "=========================" << std::endl;
	print (ss.str());

	std::shared_ptr<PairWithMatches> p1(new  PairWithMatches{im1, im2});
	DEBUG_PTR(p1);

#ifdef USE_KNN_MATCHER
    BFMatcher matcher(NORM_HAMMING);
    vector<vector<DMatch>> nn_matches;
    matcher.knnMatch(im1->desc, im2->desc, nn_matches, 2);

	const float nn_match_ratio = 0.6f;   // Nearest neighbor matching ratio
    for(auto matches : nn_matches) {
        float dist1 = matches[0].distance;
        float dist2 = matches[1].distance;

        if(dist1 < nn_match_ratio * dist2) {
            p1->matches.push_back(matches[0]);
        }
    }
#endif

#ifdef USE_GMS_FILTER
    vector<DMatch> matches_all;
    BFMatcher matcher(NORM_HAMMING);
	matcher.match(im1->desc, im2->desc, matches_all);

	// GMS filter
	std::vector<bool> vbInliers;
	gms_matcher gms(im1->kpts, im1->omni->img.size(), im2->kpts, im2->omni->img.size(), matches_all);
	int num_inliers = gms.GetInlierMask(vbInliers, false, false);
	cout << "Get total " << num_inliers << " matches." << endl;

	// collect matches
	for (size_t i = 0; i < vbInliers.size(); ++i)
	{
		if (vbInliers[i] == true)
		{
			p1->matches.push_back(matches_all[i]);
		}
	}
#endif

	return p1;
}

shared_ptr<TripletsWithMatches> commonPointsComputation (std::shared_ptr<PairWithMatches> p1, std::shared_ptr<PairWithMatches> p2) {

	std::stringstream ss {};
	ss << "=========================" << std::endl << "Common Points Computation " << p1->idString() << " " << p2->idString() <<  std::endl << "=========================" << std::endl;
	print (ss.str());

	if (p1->imgs[1]->omni->imgNum != p2->imgs[0]->omni->imgNum) {
		throw std::runtime_error ("Error in indexes in CommonPointComputation");
	}

	shared_ptr<TripletsWithMatches> t1 (new TripletsWithMatches({p1->imgs[0], p1->imgs[1], p2->imgs[1]}));
	DEBUG_PTR(t1);
	const int pairsCount = 2;
	PairWithMatches *p[pairsCount];
	p[0] = p1.get();
	p[1] = p2.get();


	for(size_t p0MatchId = 0;p0MatchId < p[0]->matches.size();p0MatchId++){
		bool ok = true;
		int matchIds[pairsCount+1];
		int nextQueryIdx = p[0]->matches[p0MatchId].trainIdx;

		matchIds[0] = p[0]->matches[p0MatchId].queryIdx;
		matchIds[1] = nextQueryIdx;
		for(size_t pxId = 1;pxId < pairsCount;pxId++){
			PairWithMatches *px = p[pxId];
			ok = false;
			for(size_t pxMatchId = 0;pxMatchId < px->matches.size();pxMatchId++){
				if(px->matches[pxMatchId].queryIdx == nextQueryIdx){
					nextQueryIdx = px->matches[pxMatchId].trainIdx;
					matchIds[pxId+1] = nextQueryIdx;
					ok = true;
					break;
				}
			}
			if(!ok) break;
		}
		if(ok){
			t1->matches.push_back(vector<int>(matchIds, matchIds + pairsCount + 1));
		}
	}


#ifdef DISPLAY_TRIPLET_MATCHES
	RNG rng(12345);
	{
		for(int repeat = 0;repeat < 1;repeat++){
			int w=t1->imgs[0]->omni->img.cols,h=t1->imgs[0]->omni->img.rows;
			Mat res(w, h, CV_8UC3, Scalar(0,0,0));
			t1->imgs[repeat]->omni->img.copyTo(res);
//			for(auto match : t1->matches){
//				circle(res, t1->imgs[0]->kpts[match[0]].pt,10,Scalar(0,255,0),2);
//			}
			for(auto match : t1->matches){
				for(int idx = 0;idx < 2;idx++){
					Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
					line(res, t1->imgs[idx]->kpts[match[idx]].pt, t1->imgs[idx + 1]->kpts[match[idx+1]].pt, color, 2);
				}
//				circle(res, t1->imgs[repeat]->kpts[match[repeat]].pt,10,Scalar(0,255,0),2);
			}
			namedWindow( "miaou", WINDOW_KEEPRATIO );
			imshow( "miaou", res);
			waitKey(0);
		}
	}
#endif
//	for(auto match : t1->matches){
//		for(int repeat = 0;repeat < 3;repeat++){
//			int w=t1->imgs[0]->omni->img.cols,h=t1->imgs[0]->omni->img.rows;
//			Mat res(w, h, CV_8UC3, Scalar(0,0,0));
//			t1->imgs[repeat]->omni->img.copyTo(res);
//			for(int idx = 0;idx < 2;idx++){
//				Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
//				line(res, t1->imgs[idx]->kpts[match[idx]].pt, t1->imgs[idx + 1]->kpts[match[idx+1]].pt, color, 2);
//			}
//
//			circle(res, t1->imgs[repeat]->kpts[match[repeat]].pt,10,Scalar(0,255,0),2);
//
//			namedWindow( "miaou", WINDOW_KEEPRATIO );
//			imshow( "miaou", res);
//			waitKey(0);
//		}
//	}

	return t1;
}



void procFeatures() {

	std::vector<std::shared_ptr<OmniWithFeatures>> v{};
	std::vector<std::shared_ptr<PairWithMatches>> lp{};
	auto mask = make_shared<Mat>();
	*mask = imread("resources/mask0.png", IMREAD_GRAYSCALE);

	for (;;) {
		std::shared_ptr<Omni> receivedPairImages { };
		receivedPairImages = imgProcQueue.wait_pop();
		std::stringstream ss {};
		ss << "=========================" << std::endl << "Received pair images " << receivedPairImages->idString() << std::endl << "=========================" << std::endl;
		print (ss.str());

		auto featuredImages = extractFeatures(receivedPairImages, mask);
		v.push_back(featuredImages);
		if (v.size() == 2) {
			lp.push_back(omniMatching (v[0], v[1]));
			v[0]=v[1];
			v.pop_back();
		}

		if (lp.size() == 2) {
			std::shared_ptr<TripletsWithMatches> p1 = commonPointsComputation (lp[0], lp[1]);
			lp[0] = lp[1];
			lp.pop_back();
			tripletsProcQueue.push(p1);
		}
	}
}

shared_ptr<Model> poseEstimation (shared_ptr<TripletsWithMatches> t1) {
	std::stringstream ss { };
	ss << "=========================" << std::endl << "Pose Estimation with triplets" << t1->idString() << std::endl
			<< "=========================" << std::endl;
	print(ss.str());

	auto m1 = make_shared<Model>();
	return m1;
}


Matx13f cross(Matx13f a, Matx13f b){
	auto c = Point3f(a(0), a(1), a(2)).cross(Point3f(b(0), b(1), b(2)));
	return Matx13f(c.x,c.y,c.z);
}

void fusionModel (Model *m1, Model *m2) {
	std::stringstream ss { };
	ss << "=========================" << std::endl << "Fusion Model (" << m2->keypoints.size() << " into " << m1->keypoints.size() << ")"
			<< std::endl
			<< "=========================" << std::endl;
	print(ss.str());
	return;

	assert(m1->keypoints.size() >= 3);
	assert(m2->keypoints.size() == 3);
	auto m1CapturesSize = m1->keypoints.size();
	auto m1C1 = m1->keypoints[m1CapturesSize-2].position;
	auto m1C2 = m1->keypoints[m1CapturesSize-1].position;
	auto m2C1 = m2->keypoints[0].position;
	auto m2C2 = m2->keypoints[1].position;
	auto m2C3 = m2->keypoints[2].position;
	auto scaleFactor = norm(m1C2-m1C1)/norm(m2C2-m2C1);
	auto t1 = (1/norm(m1C2-m1C1)) * (m1C2-m1C1);
	auto t2 = (1/norm(m2C2-m2C1)) * (m2C2-m2C1);
	auto v = cross(t1,t2);
	auto c = t1.dot(t2);
	auto w = Matx33f(0.0,-v(2),v(1),v(2),0.0,-v(0),-v(1),v(0),0.0);
	auto rotation = Matx33f().eye() + w + (1/(1+c))*(w*w);
	auto translation = m1C1-m2C1;

	for(auto source : m2->features){
		auto feature = ModelFeature();
		feature.color = source.color;
		feature.position = translation + scaleFactor * (source.position* rotation);
		m1->features.push_back(feature);
	}
	ModelKeypoint m1C3;
	m1C3.position = translation + scaleFactor * (m2C3*rotation);
	m1->keypoints.push_back(m1C3);
}


void writePly(const std::string &file, const Vec_Points<double> &features){
//	‘__gnu_cxx::__normal_iterator<ModelKeypoint*, std::vector<ModelKeypoint> >
	std::ofstream s {};
	s.open (file);

	s << "ply" << std::endl;
	s << "format ascii 1.0 " << std::endl;
	s << "element vertex " << features.size() << std::endl;
	s << "comment " << file << std::endl;
	s << "property float32 x " << std::endl;
	s << "property float32 y " << std::endl;
	s << "property float32 z " << std::endl;
	s << "end_header " << std::endl;

	for(size_t i {0}; i < features.size(); ++i){
		Points<double> f = features[i];
		s << f[0] << " " << f[1] << " " << f[2] << std::endl;
	}

	s.close();
}

void ProcPose() {

	std::vector <Model> vecm {};
	Model m {};

	size_t counter {0};

	for (;;) {
		std::shared_ptr<TripletsWithMatches> receivedTripletsImages { };
		receivedTripletsImages = tripletsProcQueue.wait_pop();

		std::vector<Vec_Points<double>> p3d_liste;

		auto width = receivedTripletsImages->imgs[0]->omni->img.cols;
		auto height = receivedTripletsImages->imgs[0]->omni->img.rows;

		for(int idx = 0;idx < 3;idx++){
			Vec_Points<double> list_matches {};
			for(auto match : receivedTripletsImages->matches){
				auto xy = receivedTripletsImages->imgs[idx]->kpts[match[idx]].pt;
				/* convert x and y to spherical */

				double theta { xy.x / (width) * 2*M_PI };
				double phi { xy.y / (height) * M_PI };
				double x { sin(theta) * sin(phi) };
				double y { -cos(phi) };
				double z { cos(theta) * sin(phi) };
				Points<double> p {x,y,z};
				list_matches.push_back(p);
			}
			std::string plyFileNameOri { "./resources/models_ori_" + std::to_string(idx) + "_" + std::to_string(counter) + ".ply"};
			writePly(plyFileNameOri, list_matches);

			p3d_liste.push_back(list_matches);
		}

		Vec_Points<double> sv_scene{};
		std::vector<Points<double>> positions {};

		double error_max { 1e-8 };

		pose_estimation (p3d_liste, error_max, sv_scene, positions);

		std::string plyFileName { "./resources/models_est_" + std::to_string(counter) + ".ply"};
		counter++;
		writePly(plyFileName, sv_scene);

		std::stringstream ss {};
		ss << "=========================" << std::endl << "Received triplets images " << receivedTripletsImages->idString() << std::endl << "=========================" << std::endl;
		print (ss.str());

//		Triplets t1 {receivedTripletsImages->getListIdx1(), receivedTripletsImages->getListIdx2(), receivedTripletsImages->getListIdx3()};
		auto m1 = poseEstimation (receivedTripletsImages);

		fusionModel (&m, m1.get());
	}
}


int main() {


	std::thread GenPairs (generatePairImages);
	std::thread ProcessFeatureExtraction (procFeatures);
	std::thread ProcessPoseEstimation (ProcPose);

	GenPairs.join();
	ProcessFeatureExtraction.join();
	ProcessPoseEstimation.join();

	return 0;

}






