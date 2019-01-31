#ifndef SRC_CONFIG_HPP_
#define SRC_CONFIG_HPP_


//#define USE_ORB_FEATURE
#define USE_AKAZE_FEATURE

//#define USE_KNN_MATCHER
#define USE_GMS_FILTER

//#define DISPLAY_TRIPLET_MATCHES
//#define DISPLAY_TRIPLET_MATCHES_INDIVIDUAL

// Folder names for the data
// input main folder
const std::string inputFolder { "data_in" };
// input folder where the input images are located
const std::string inputDataSet { "0_dataset" };
// input folder where the mask is located
const std::string inputMask { "1_maksk" };

// output main folder
const std::string outputFolder { "data_out" };
// output folder where the features for each image is written
const std::string outputFeatures { "1_features" };
// output folder where the matches for each pair of image is written
const std::string outputMatches { "2_matches" };
// output folder where the triplets matches for two pairs of images is written
const std::string outputTriplets { "3_triplets" };
// output folder where the spherical coordinates of the triplets are written
const std::string outputSpherical { "4_spherical" };



#endif /* SRC_CONFIG_HPP_ */
