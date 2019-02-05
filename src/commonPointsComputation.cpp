#include "pipelineAlgo.hpp"

shared_ptr<TripletsWithMatches> commonPointsComputation (std::shared_ptr<PairWithMatches> p1, std::shared_ptr<PairWithMatches> p2) {

	// Print out message on the console
	std::stringstream ss {};
	ss << "=========================" << std::endl
	   << "Common Points Computation " << "(" << p1->getImageNumber1() << ", " << p1->getImageNumber2() << ") (" << p2->getImageNumber1() << ", " << p2->getImageNumber2() << ")" << std::endl
	   << "=========================" << std::endl;
	print (ss.str());

	// Check if the pairs are consecutive
	if (p1->getImageNumber2() != p2->getImageNumber1()) {
		throw std::runtime_error ("Error in indexes in CommonPointComputation");
	}

	shared_ptr<TripletsWithMatches> t1(new TripletsWithMatches { p1->getImage1(), p1->getImage2(), p2->getImage2() });

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


#ifdef DISPLAY_TRIPLET_MATCHES
	{
		RNG rng(12345);
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

#ifdef DISPLAY_TRIPLET_MATCHES_INDIVIDUAL
	{
		RNG rng(12345);
		for(auto match : t1->matches){
			for(int repeat = 0;repeat < 3;repeat++){
				int w=t1->imgs[0]->omni->img.cols,h=t1->imgs[0]->omni->img.rows;
				Mat res(w, h, CV_8UC3, Scalar(0,0,0));
				t1->imgs[repeat]->omni->img.copyTo(res);
				for(int idx = 0;idx < 2;idx++){
					Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
					line(res, t1->imgs[idx]->kpts[match[idx]].pt, t1->imgs[idx + 1]->kpts[match[idx+1]].pt, color, 2);
				}

				circle(res, t1->imgs[repeat]->kpts[match[repeat]].pt,10,Scalar(0,255,0),2);

				namedWindow( "miaou", WINDOW_KEEPRATIO );
				imshow( "miaou", res);
				waitKey(0);
			}
		}
	}
#endif

	return t1;
}
