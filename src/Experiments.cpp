////============================================================================
//// Name        : Processor.cpp
//// Author      : Marcelo E. Kaihara
//// Version     :
//// Copyright   :
//// Description : Hello World in C++, Ansi-style
////============================================================================
//
//#include <iostream>
//#include <thread>
//#include <mutex>
//#include <sstream>
//#include <iostream>
//#include <fstream>
//#include <iterator>
//#include <vector>
//#include <opencv4/opencv2/core/types.hpp>
//#include <opencv4/opencv2/core.hpp>
//#include <opencv4/opencv2/features2d.hpp>
//#include <opencv4/opencv2/imgcodecs.hpp>
//#include <opencv4/opencv2/opencv.hpp>
//
//#include "ctpl.hpp"
//#include "Queue.hpp"
//
//#include "gms_matcher.h"
//
//
//
//#include <opencv4/opencv2/features2d.hpp>
//#include <opencv4/opencv2/imgcodecs.hpp>
//#include <opencv4/opencv2/opencv.hpp>
//#include <vector>
//#include <iostream>
//
//using namespace std;
//using namespace cv;
//
//
////#define DO_AKAZE
////#define DO_MATCH_AKAZE
//#define DO_ORB
//#define DO_GMS
//
//
//class Toto{
//public:
//	Toto(int a) { cout << "Toto(int)" << a << endl; };
//	Toto() { cout << "Toto()" << endl; };
//	Toto(Toto &a) { cout << "Toto(Toto &)" << endl; };
//	Toto(Toto &&a) { cout << "Toto(Toto &&)" << endl; };
//	~Toto()  { cout << "~Toto()" << endl; };
//
//
//};
//
//int main3(){
////	Toto x = Toto(1);
//	auto a = make_shared<Toto>(Toto(1));
//
//	auto b = make_shared<Toto>(2);
//	auto c = shared_ptr<Toto>(new Toto(3));
//
//}
//
//int main(){
//	//const float inlier_threshold = 2.5f; // Distance threshold to identify inliers
//
//	Rect myROI(0, 0, 6000, 3000);
//    Mat img1 = imread("resources/20181010-144454-906636.png", IMREAD_UNCHANGED)(myROI);
//    Mat img2 = imread("resources/20181010-144504-156640.png", IMREAD_UNCHANGED)(myROI);
//    Mat mask1 = imread("resources/mask0.png", IMREAD_GRAYSCALE)(myROI);
//
//    vector<KeyPoint> kpts1, kpts2;
//    Mat desc1, desc2;
//
//
//    for(int y = 0; y < myROI.height;y++){
//        for(int x = 0; x < myROI.width;x++){
//        	auto read = mask1.at<uint8_t>(y,x);
//        	mask1.at<uint8_t>(y,x) = mask1.at<uint8_t>(y,x) == 0xFF ? 1 : 0;
//        }
//    }
//
//    auto mask2 = mask1;
//
//#ifdef DO_AKAZE
//    cout << "DO_AKAZE" << endl;
//    Ptr<AKAZE> akaze = AKAZE::create(
//		AKAZE::DESCRIPTOR_MLDB,
//		0,  3,
//		0.0001f,  4,
//		4, KAZE::DIFF_PM_G2
//	);
//    akaze->detectAndCompute(img1, mask1, kpts1, desc1);
//    akaze->detectAndCompute(img2, mask2, kpts2, desc2);
//#endif
//
//#ifdef DO_ORB
//    cout << "DO_ORB" << endl;
//	Ptr<ORB> orb = ORB::create(100000);
//	orb->setFastThreshold(0);
//
//	orb->detectAndCompute(img1, mask1, kpts1, desc1);
//	orb->detectAndCompute(img2, mask2, kpts2, desc2);
//#endif
//
//	cout << "keypoints : " << kpts1.size() << " + " << kpts2.size() << endl;
//
//#ifdef DO_MATCH_AKAZE
//    cout << "BFMatcher" << endl;
//	const float nn_match_ratio = 0.6f;   // Nearest neighbor matching ratio
//
//    BFMatcher matcher(NORM_HAMMING);
//    vector<vector<DMatch>> nn_matches;
//    matcher.knnMatch(desc1, desc2, nn_matches, 2);
//
//    vector<KeyPoint> matched1, matched2, inliers1, inliers2;
//    vector<DMatch> good_matches;
//    for(size_t i = 0; i < nn_matches.size(); i++) {
//        DMatch first = nn_matches[i][0];
//        float dist1 = nn_matches[i][0].distance;
//        float dist2 = nn_matches[i][1].distance;
//
//        if(dist1 < nn_match_ratio * dist2) {
//            int new_i = static_cast<int>(matched1.size());
//            matched1.push_back(kpts1[first.queryIdx]);
//            matched2.push_back(kpts2[first.trainIdx]);
//            good_matches.push_back(DMatch(first.queryIdx,first.trainIdx, 0));
//        }
//    }
//#endif
//
//
//#ifdef DO_GMS
//    vector<DMatch> matches_all;
//    BFMatcher matcher(NORM_HAMMING);
//	matcher.match(desc1, desc2, matches_all);
//
//	// GMS filter
//	std::vector<bool> vbInliers;
//	gms_matcher gms(kpts1, img1.size(), kpts2, img2.size(), matches_all);
//	int num_inliers = gms.GetInlierMask(vbInliers, false, false);
//	cout << "Get total " << num_inliers << " matches." << endl;
//
//    vector<DMatch> good_matches;
//	// collect matches
//	for (size_t i = 0; i < vbInliers.size(); ++i)
//	{
//		if (vbInliers[i] == true)
//		{
//			good_matches.push_back(matches_all[i]);
//		}
//	}
//#endif
//    cout << "Matches : " << good_matches.size() << endl;
//
////    for(unsigned i = 0; i < matched1.size(); i++) {
//////        Mat col = Mat::ones(3, 1, CV_64F);
//////        col.at<double>(0) = matched1[i].pt.x;
//////        col.at<double>(1) = matched1[i].pt.y;
//////
//////        col = homography * col;
//////        col /= col.at<double>(2);
//////        double dist = sqrt( pow(col.at<double>(0) - matched2[i].pt.x, 2) +
//////                            pow(col.at<double>(1) - matched2[i].pt.y, 2));
////
//////        if(dist < inlier_threshold) {
////            int new_i = static_cast<int>(inliers1.size());
////            inliers1.push_back(matched1[i]);
////            inliers2.push_back(matched2[i]);
////            good_matches.push_back(DMatch(new_i, new_i, 0));
//////        }
////    }
//
//
//
//    {
//        RNG rng(12345);
//		Mat res(myROI.width,myROI.height, CV_8UC3, Scalar(0,0,0));
//		res = img1;// - Scalar(255,0,255);
////		res += img2 - Scalar(0,255,255);
//		for(auto m : good_matches){
//			Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
//			line(res, kpts1[m.queryIdx].pt,  kpts2[m.trainIdx].pt, color, 2);
//		}
//		namedWindow( "miaou", WINDOW_KEEPRATIO );
//		imshow( "miaou", res);
//		waitKey(0);
//    }
//
//    {
//        RNG rng(12345);
//		Mat res(myROI.width,myROI.height, CV_8UC3, Scalar(0,0,0));
//		res = img2;// - Scalar(255,0,255);
////		res += img2 - Scalar(0,255,255);
//		for(auto m : good_matches){
//			Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
//			line(res, kpts1[m.queryIdx].pt,  kpts2[m.trainIdx].pt, color, 2);
//		}
//		namedWindow( "miaou", WINDOW_KEEPRATIO );
//		imshow( "miaou", res);
//		waitKey(0);
//    }
////    for(int offset = 0;offset < good_matches.size() - 10;offset += 100){
////        RNG rng(12345);
////		Mat res(myROI.width,myROI.height, CV_8UC3, Scalar(0,0,0));
////		res += img1 - Scalar(255,0,255);
////		res += img2 - Scalar(0,255,255);
////		for(unsigned i = 0; i < 10; i++) {
////			auto m = good_matches[i+offset];
////			Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
////			line(res, kpts1[m.queryIdx].pt,  kpts2[m.trainIdx].pt, color, 10);
////		}
////		namedWindow( "miaou", WINDOW_KEEPRATIO );
////		imshow( "miaou", res);
////		waitKey(0);
////    }
//
//
//    {
//		Mat res;
//		drawMatches(img1, kpts1, img2, kpts2, good_matches, res);
//		namedWindow( "miaou", WINDOW_KEEPRATIO );
//		imshow( "miaou", res);
//		waitKey(0);
//		imwrite("res.png", res);
//    }
//
//    for(int offset = 0;offset < good_matches.size() - 10;offset += 10){
//    	{
//			RNG rng(offset);
//			Mat res(myROI.width,myROI.height, CV_8UC3, Scalar(0,0,0));
//			res += img1;
//			for(unsigned i = 0; i < 10; i++) {
//				auto m = good_matches[i+offset];
//				Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
//				line(res, kpts1[m.queryIdx].pt, kpts2[m.trainIdx].pt, color, 2);
//				circle(res, kpts2[m.trainIdx].pt,10,color,2);
//			}
//			namedWindow( "miaou", WINDOW_KEEPRATIO );
//			imshow( "miaou", res);
//			waitKey(0);
//    	}
//
//    	{
//			RNG rng(offset);
//			Mat res(myROI.width,myROI.height, CV_8UC3, Scalar(0,0,0));
//			res += img2;
//			for(unsigned i = 0; i < 10; i++) {
//				auto m = good_matches[i+offset];
//				Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
//				line(res, kpts1[m.queryIdx].pt, kpts2[m.trainIdx].pt, color, 2);
//			}
//			namedWindow( "miaou", WINDOW_KEEPRATIO );
//			imshow( "miaou", res);
//			waitKey(0);
//    	}
//    }
//
//    for(int offset = 0;offset < good_matches.size() - 10;offset += 10){
//		Mat res;
//		auto matches = vector<DMatch>(good_matches.begin() + offset, good_matches.begin() + offset + 10);
//		drawMatches(img1, kpts1, img2, kpts2, matches, res);
//		namedWindow( "miaou", WINDOW_KEEPRATIO );
//		imshow( "miaou", res);
//		waitKey(0);
//		imwrite("res.png", res);
//    }
////    double inlier_ratio = inliers1.size() * 1.0 / matched1.size();
////    cout << "A-KAZE Matching Results" << endl;
////    cout << "*******************************" << endl;
////    cout << "# Keypoints 1:                        \t" << kpts1.size() << endl;
////    cout << "# Keypoints 2:                        \t" << kpts2.size() << endl;
////    cout << "# Matches:                            \t" << matched1.size() << endl;
////    cout << "# Inliers:                            \t" << inliers1.size() << endl;
////    cout << "# Inliers Ratio:                      \t" << inlier_ratio << endl;
////    cout << endl;
//
//    return 0;
//}
//
//
//
//
