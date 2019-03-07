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
const std::string inputDataSet { "0_dataset_20181218-160912_SionOutdoor" };
//const std::string inputDataSet { "0_dataset_20180904-164403_HESOutdoorCircular" };
//const std::string inputDataSet { "0_dataset_20180904-164727_HESOutdoorLinear" };
//const std::string inputDataSet { "0_dataset_20181010-144454_HESIndoorOffice" };



// input folder where the mask is located
const std::string inputMask { "1_mask" };
// name of the file of the mask
const std::string inputMaskFileName { "mask0.png" };

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
// output folder where the rotation and translation of triplets are written
const std::string outputPose3 { "5_pose_3" };
// output folder where the point cloud of the triplets are written
const std::string outputPointCloud3 { "6_sparse_3" };
// output folder where the odometry results are written
const std::string outputOdometry { "7_odometry" };
// output folder where the progressive model is written
const std::string outputProgressiveModel { "8_models" };
// output file name of the merged model
const std::string outputMergedModelFileName { "sparse.ply" };
// output folder where the filtered triplets matches (after pose estimation) for two pairs of images is written
const std::string outputTripletsFiltered { "3_triplets_filtered" };


#endif /* SRC_CONFIG_HPP_ */
