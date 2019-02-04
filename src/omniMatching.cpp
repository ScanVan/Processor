#include "pipelineAlgo.hpp"

#include "gms_matcher.h"

std::shared_ptr<PairWithMatches> omniMatching (std::shared_ptr<EquirectangularWithFeatures> im1, std::shared_ptr<EquirectangularWithFeatures> im2) {
// Calculates the matching between the features of two images


	// Print message on the screen
	std::stringstream ss {};
	ss << "=========================" << std::endl
	   << "Feature Extraction (" << im1->idString() << " "  << im2->idString() << ")" << std::endl
	   << "=========================" << std::endl;
	print (ss.str());

	// Shared pointer of the object that stores the matches
	std::shared_ptr<PairWithMatches> p1(new  PairWithMatches{im1, im2});

	//DEBUG_PTR(p1);

#ifdef USE_KNN_MATCHER
    BFMatcher matcher(NORM_HAMMING);
    vector<vector<DMatch>> nn_matches;
    matcher.knnMatch(im1->desc, im2->desc, nn_matches, 2);

	const float nn_match_ratio = 0.8f;   // Nearest neighbor matching ratio
    for(auto matches : nn_matches) {
        float dist1 = matches[0].distance;
        float dist2 = matches[1].distance;

        if(dist1 < nn_match_ratio * dist2) {
            p1->matches.push_back(matches[0]);
        }
    }
#endif

#ifdef USE_GMS_FILTER
    std::vector<cv::DMatch> matches_all;
    cv::BFMatcher matcher(NORM_HAMMING);
	matcher.match(im1->getDesc(), im2->getDesc(), matches_all);

	// GMS filter
	std::vector<bool> vbInliers;
	gms_matcher gms(im1->getKeyPoints(), (im1->getOmni()->getImage()).size(), im2->getKeyPoints(), (im2->getOmni()->getImage()).size(), matches_all);
	int num_inliers = gms.GetInlierMask(vbInliers, false, false);
	cout << "Get total " << num_inliers << " matches." << endl;

	// collect matches
	for (size_t i = 0; i < vbInliers.size(); ++i)
	{
		// if inliner, then push into valid matches
		if (vbInliers[i] == true)
		{
			p1->getMatches().push_back(matches_all[i]);
		}
	}

//	cv::Mat outImg{};
//
//	cv::drawMatches(p1->getImage1()->getOmni()->getImage(), p1->getKeyPoints1(), p1->getImage2()->getOmni()->getImage(), p1->getKeyPoints2(), p1->getMatches(), outImg);
//
//	namedWindow("Matches", cv::WINDOW_NORMAL);
//	imshow ("Matches", outImg);
//	waitKey(0);


#endif

	return p1;
}

