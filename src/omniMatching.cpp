#include "pipelineAlgo.hpp"
#include "gms_matcher.h"


bool movementCheck (std::shared_ptr<PairWithMatches> matches, double threshold) {
// Checks if the pair of images are in movement
// Inputs:
// matches: shared pointer to PairWithMatches
// threshold: the threshold to check if the pair is in movement
// Output:
// true if movement is detected, false if it is in still mode

	if (matches->computeStillDistance() > threshold) {
		return true;
	}

	return false;

}

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
    matcher.knnMatch(im1->getDesc(), im2->getDesc(), nn_matches, 2);

	const float nn_match_ratio = 0.8f;   // Nearest neighbor matching ratio
    for(auto matches : nn_matches) {
        float dist1 = matches[0].distance;
        float dist2 = matches[1].distance;

        if(dist1 < nn_match_ratio * dist2) {
            p1->getMatches().push_back(matches[0]);
        }
    }
#endif

#ifdef USE_GMS_FILTER
    std::vector<cv::DMatch> matches_all;
    cv::BFMatcher matcher(NORM_HAMMING, true);
    // Second param is boolean variable, crossCheck which is false by default. If it is true, Matcher returns only those matches with value (i,j) such that i-th descriptor in set A has j-th descriptor in set B as the best match and vice-versa. That is, the two features in both sets should match each other. It provides consistent result, and is a good alternative to ratio test proposed by D.Lowe in SIFT paper.
	matcher.match(im1->getDesc(), im2->getDesc(), matches_all);

	// GMS filter
	std::vector<bool> vbInliers;
	gms_matcher gms(im1->getKeyPoints(), (im1->getOmni()->getImage()).size(), im2->getKeyPoints(), (im2->getOmni()->getImage()).size(), matches_all);
	//int num_inliers = gms.GetInlierMask(vbInliers, false, false);
	gms.GetInlierMask(vbInliers, false, false);

	// collect matches
	for (size_t i = 0; i < vbInliers.size(); ++i)
	{
		// if inliner, then push into valid matches
		if (vbInliers[i] == true)
		{
			p1->getMatches().push_back(matches_all[i]);
		}
	}

	cout << "Get total " << p1->getMatches().size() << " matches." << endl;


	/*cv::Mat outImg{};

	cv::drawMatches(p1->getImage1()->getOmni()->getImage(), p1->getKeyPoints1(), p1->getImage2()->getOmni()->getImage(), p1->getKeyPoints2(), p1->getMatches(), outImg);

	std::string filename {p1->getImage1()->getOmni()->getImgName() + "_" +p1->getImage2()->getOmni()->getImgName() + ".jpg" };
	cv::imwrite(filename, outImg);
*/

	//namedWindow("Matches", cv::WINDOW_NORMAL);
	//imshow ("Matches", outImg);
	//waitKey(0);


#endif

	return p1;
}

