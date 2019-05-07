#include "pipelineAlgo.hpp"

class KeyPointPosition {
public:
	cv::Point2f p { };
	KeyPointPosition( cv::Point2f &p ): p {p} {}
	bool operator< (const KeyPointPosition a) const {
		std::stringstream s1{};
		s1 << std::setprecision(15) << p.x << p.y;
		std::stringstream s2{};
		s2 << std::setprecision(15) << a.p.x << a.p.y;
		return (s1.str() < s2.str());

	}
};

std::shared_ptr<TripletsWithMatches> commonPointsComputation (std::shared_ptr<PairWithMatches> p1, std::shared_ptr<PairWithMatches> p2) {


	// Print out message on the console
	std::stringstream ss {};
	ss << "=========================" << std::endl
	   << "Common Points Computation " << "(" << p1->getImageNumber1() << ", " << p1->getImageNumber2() << ") (" << p2->getImageNumber1() << ", " << p2->getImageNumber2() << ")" << std::endl
	   << "=========================" << std::endl;
	print (ss.str());

	// Check if the pairs are consecutive
//	if (p1->getImageNumber2() != p2->getImageNumber1()) {
//		throw std::runtime_error ("Error in indexes in CommonPointComputation");
//	}

	std::shared_ptr<TripletsWithMatches> t1(new TripletsWithMatches { p1->getImage1(), p1->getImage2(), p2->getImage2() });

	//DEBUG_PTR(t1);

	// number of pairs of images to check the common keypoints
	// in case of triplets, it compares 2 images
	const int pairsCount { 2 };

	// vector of pointers to PairWithMatches
	PairWithMatches *p[pairsCount] { };
	p[0] = p1.get(); // pointer to PairWithMatches of the first pair
	p[1] = p2.get(); // pointer to PairWithMatches of the second pair

	// if there are matches between pairs the compute the common matches for the triplet
	if ((p[0]->getMatches().size() != 0) || (p[1]->getMatches().size() != 0)) {

		// loop over the matches of the first pair
		for (size_t p0MatchId { 0 }; p0MatchId < p[0]->getMatches().size(); ++p0MatchId) {

			bool ok { true };
			// matchIds stores the keypoint index of the matches
			// for triples the size is 3
			int matchIds[pairsCount + 1] { };
			// nextQueryIdx is the index of the keypoints of the second image of the first pair
			int nextQueryIdx { p[0]->getMatches()[p0MatchId].trainIdx };

			// matchIds[0] is the index of the keypoints of the first image of the first pair
			matchIds[0] = p[0]->getMatches()[p0MatchId].queryIdx;
			// matchIds[1] is the index of the keypoints of the second image of the first pair
			matchIds[1] = nextQueryIdx;

			// for the case of triplets it loops over the list of matches of the second pair
			for (size_t pxId { 1 }; pxId < pairsCount; ++pxId) {

				PairWithMatches *px = p[pxId]; // in this case is the pointer to PairWithMatches of the second pair

				ok = false;

				// loop over the list of matches of the second pair
				for (size_t pxMatchId { 0 }; pxMatchId < px->getMatches().size(); ++pxMatchId) {

					// if the keypoint index of the first image of the second pair is equal to
					// the keypoint index of the second image of the first pair
					if (px->getMatches()[pxMatchId].queryIdx == nextQueryIdx) {
						// then stores this index in matchId[3] and exit the loop
						nextQueryIdx = px->getMatches()[pxMatchId].trainIdx;
						matchIds[pxId + 1] = nextQueryIdx;
						ok = true;
						break;
					}
				}
				// in case of more than a triplet, if it didn't find the correspondence on the list
				// then exit the loop and does not check the other list of mathes
				if (!ok)
					break;

			}
			// if the triplet was found then pushes to the vector of matches in the triplet
			if (ok) {
				t1->getMatchVector().push_back(std::vector<int>(matchIds, matchIds + pairsCount + 1));
			}
		}
	}

	// Calculates the frequency of usage of the keypoints

	std::map<KeyPointPosition, int> k1 { };
	std::map<KeyPointPosition, int> k2 { };
	std::map<KeyPointPosition, int> k3 { };

	for (auto match: t1->getMatchVector()) {
		auto kp1 = t1->getImage()[0]->getKeyPoints()[match[0]].pt;
		auto kp2 = t1->getImage()[1]->getKeyPoints()[match[1]].pt;
		auto kp3 = t1->getImage()[2]->getKeyPoints()[match[2]].pt;
		auto it = k1.find(kp1);
		if (it!=k1.end()) {
			k1[kp1]++;
		} else {
			k1[kp1] = 1;
		}
		auto it2 = k2.find(kp2);
		if (it2 != k2.end()) {
			k2[kp2]++;
		} else {
			k2[kp2] = 1;
		}
		auto it3 = k3.find(kp3);
		if (it3 != k3.end()) {
			k3[kp3]++;
		} else {
			k3[kp3] = 1;
		}
	}

	std::vector<int> fq1{};
	std::vector<int> fq2{};
	std::vector<int> fq3{};
	for (const auto &k:k1) {
		fq1.push_back(k.second);
	}
	for (const auto &k : k2) {
		fq2.push_back(k.second);
	}
	for (const auto &k : k3) {
		fq3.push_back(k.second);
	}
	std::sort(fq1.begin(), fq1.end(), std::greater<int>());
	std::sort(fq2.begin(), fq2.end(), std::greater<int>());
	std::sort(fq3.begin(), fq3.end(), std::greater<int>());

//	for (const auto &x : fq1) {
//		std::cout << x << " ";
//	}
//	std::cout << std::endl << std::endl;
//
//	for (const auto &x : fq2) {
//		std::cout << x << " ";
//	}
//	std::cout << std::endl << std::endl;
//
//	for (const auto &x : fq3) {
//		std::cout << x << " ";
//	}
//	std::cout << std::endl << std::endl;
//
//	std::cout << std::accumulate(fq1.begin(), fq1.end(), 0.0) << std::endl;
//	std::cout << std::accumulate(fq2.begin(), fq2.end(), 0.0) << std::endl;
//	std::cout << std::accumulate(fq3.begin(), fq3.end(), 0.0) << std::endl;

	// stores the result in the Triplet
	t1->setFrequencyMatches1(fq1);
	t1->setFrequencyMatches2(fq2);
	t1->setFrequencyMatches3(fq3);

#ifdef DISPLAY_TRIPLET_MATCHES
	{
		cv::RNG rng(12345);
		for(int repeat = 0;repeat < 3;repeat++){
			int w { t1->getImage()[0]->getOmni()->getImage().cols }, h { t1->getImage()[0]->getOmni()->getImage().rows };
			cv::Mat res(w, h, CV_8UC3, cv::Scalar(0,0,0));
			t1->getImage()[repeat]->getOmni()->getImage().copyTo(res);
//			for(auto match : t1->matches){
//				circle(res, t1->imgs[0]->kpts[match[0]].pt,10,Scalar(0,255,0),2);
//			}
			for(auto match : t1->getMatchVector()){
				cv::Scalar color = cv::Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
				for(int idx = 0;idx < 2;idx++){
					cv::line(res, t1->getImage()[idx]->getKeyPoints()[match[idx]].pt, t1->getImage()[idx + 1]->getKeyPoints()[match[idx+1]].pt, color, 2);
				}
				cv::circle(res, t1->getImage()[repeat]->getKeyPoints()[match[repeat]].pt,10,cv::Scalar(0,255,0),2);
			}

			std::string filename {t1->getImage()[0]->getOmni()->getImgName() + "_" +t1->getImage()[1]->getOmni()->getImgName() + "_" + t1->getImage()[2]->getOmni()->getImgName() + ".jpg" };
			cv::imwrite(filename, res);
			cv::namedWindow( "Matches", cv::WINDOW_KEEPRATIO );
			cv::imshow( "Matches", res);
			cv::waitKey(0);
		}
	}

#endif

#ifdef DISPLAY_TRIPLET_MATCHES_INDIVIDUAL
	{
		cv::RNG rng(12345);
		for(auto match : t1->getMatchVector()){
			for(int repeat = 0;repeat < 3;repeat++){
				int w {t1->getImage()[0]->getOmni()->getImage().cols}, h {t1->getImage()[0]->getOmni()->getImage().rows};
				cv::Mat res(w, h, CV_8UC3, cv::Scalar(0,0,0));
				t1->getImage()[repeat]->getOmni()->getImage().copyTo(res);
				for(int idx = 0;idx < 2;idx++){
					cv::Scalar color = cv::Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
					cv::line(res, t1->getImage()[idx]->getKeyPoints()[match[idx]].pt, t1->getImage()[idx + 1]->getKeyPoints()[match[idx+1]].pt, color, 2);
				}

				cv::circle(res, t1->getImage()[repeat]->getKeyPoints()[match[repeat]].pt,10,cv::Scalar(0,255,0),2);

				cv::namedWindow( "Matches", cv::WINDOW_KEEPRATIO );
				cv::imshow( "Matches", res);
				cv::waitKey(0);
			}
		}
	}
#endif

	return t1;
}
