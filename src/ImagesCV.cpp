#include "ImagesCV.hpp"

namespace ScanVan {

ImagesCV::ImagesCV(): Images() {
	p_openCvImage = new cv::Mat {};
}

ImagesCV::ImagesCV(ImagesRaw &img): Images{} {

	cv::Mat openCvImageRG8 = cv::Mat(height, width, CV_8UC1, img.getBufferP());

	p_openCvImage = new cv::Mat{};

	cv::cvtColor(openCvImageRG8, *p_openCvImage, cv::COLOR_BayerRG2RGB);

	height = img.getHeight();
	width = img.getWidth();
	cameraIdx = img.getCameraIdx();	// camera index
	captureTimeCPUStr = img.getCaptureCPUTime(); // capture time taken on the CPU in string format
	captureTimeCamStr = img.getCaptureCamTime(); // trigger time retrieved from the camera, number of ticks, string format
	exposureTime = img.getExposureTime();// exposure time
	gain = img.getGain();		// gain
	balanceR = img.getBalanceR();	// white balance R
	balanceG = img.getBalanceG();	// white balance G
	balanceB = img.getBalanceB();	// white balance B
	autoExpTime = img.getAutoExpTime();	// Auto Exposure Time
	autoGain = img.getAutoGain(); 		// Auto Gain
	numImages = img.getImgNumber();
	serialNum = img.getSerialNumber();
}

ImagesCV::ImagesCV(ImagesCV &img): Images{} {

	p_openCvImage = new cv::Mat{*(img.p_openCvImage)};

	//openCvImage = img.openCvImage.clone();

	height = img.getHeight();
	width = img.getWidth();
	cameraIdx = img.getCameraIdx();	// camera index
	captureTimeCPUStr = img.getCaptureCPUTime(); // capture time taken on the CPU in string format
	captureTimeCamStr = img.getCaptureCamTime(); // trigger time retrieved from the camera, number of ticks, string format
	exposureTime = img.getExposureTime();// exposure time
	gain = img.getGain();		// gain
	balanceR = img.getBalanceR();	// white balance R
	balanceG = img.getBalanceG();	// white balance G
	balanceB = img.getBalanceB();	// white balance B
	autoExpTime = img.getAutoExpTime();	// Auto Exposure Time
	autoGain = img.getAutoGain(); 		// Auto Gain
	numImages = img.getImgNumber();
	serialNum = img.getSerialNumber();
}

ImagesCV::ImagesCV(ImagesCV &&img): Images{} {

	p_openCvImage = img.p_openCvImage;
	img.p_openCvImage = nullptr;

	height = img.getHeight();
	width = img.getWidth();
	cameraIdx = img.getCameraIdx();	// camera index
	captureTimeCPUStr = img.getCaptureCPUTime(); // capture time taken on the CPU in string format
	captureTimeCamStr = img.getCaptureCamTime(); // trigger time retrieved from the camera, number of ticks, string format
	exposureTime = img.getExposureTime();// exposure time
	gain = img.getGain();		// gain
	balanceR = img.getBalanceR();	// white balance R
	balanceG = img.getBalanceG();	// white balance G
	balanceB = img.getBalanceB();	// white balance B
	autoExpTime = img.getAutoExpTime();	// Auto Exposure Time
	autoGain = img.getAutoGain(); 		// Auto Gain
	numImages = img.getImgNumber();
	serialNum = img.getSerialNumber();
}



void ImagesCV::show () const {
	/// Display
	cv::namedWindow("Image", cv::WINDOW_NORMAL);
	cv::imshow("Image", *p_openCvImage);
};

void ImagesCV::show(std::string name) const {
	/// Display
	cv::namedWindow(name, cv::WINDOW_NORMAL);
	cv::imshow(name, *p_openCvImage);
};

void ImagesCV::showConcat (std::string name, Images &img2) const {

	cv::Mat m;
	try {
		cv::hconcat(*p_openCvImage, *(dynamic_cast<ImagesCV &>(img2).p_openCvImage), m);
	} catch (...) {
		m = *p_openCvImage;
	}

	/// Display
	cv::namedWindow(name, cv::WINDOW_NORMAL);
	cv::imshow(name, m);
}

void ImagesCV::remap (const cv::Mat & map_1, const cv::Mat & map_2) {

	cv::Mat * undistorted = new cv::Mat{};
	cv::Mat * old_p_img = p_openCvImage;

	// main remapping function that undistort the images
	cv::remap(*p_openCvImage, *undistorted, map_1, map_2, cv::INTER_CUBIC, cv::BORDER_CONSTANT);

	p_openCvImage = undistorted;
	delete old_p_img;
}

void ImagesCV::saveImage (std::string path) {
	// It saves the object's image to file
	// It saves in bmp format
	std::string ext = path.substr(path.find_last_of(".") + 1);

	// Save the raw image into file
	if (ext == "raw") {
		throw std::runtime_error("Tried to save file in raw format from object ImagesCV");
	} else if (ext == "bmp") {
		try {
			imwrite(path, *p_openCvImage);
		} catch (std::exception & ex) {
			std::cerr << "Error writing the bmp file: " << ex.what() << std::endl;
			throw ex;
		}
	} else {
		throw std::runtime_error("File extension not recognized when trying to save the image from ImagesCV.");
	}
}

void ImagesCV::saveData(std::string path) {
	// Saves the opencv image and the camera data to file
	// Here path is the path to the directory where the images will be stored.
	// The image number and the camera index are extracted from the object.
	// The function will automatically add the .bmp for the opencv data image and .txt for the camera
	// configuration.


	std::stringstream ss1 { };

	ss1 << path;
	ss1 << cameraIdx;
	ss1 << "_";
	ss1 << numImages;
	ss1 << ".bmp";

//	std::stringstream ss2 { };
//
//	ss2 << path;
//	ss2 << "img_";
//	ss2 << cameraIdx;
//	ss2 << "_";
//	ss2 << numImages;
//	ss2 << "_bmp";
//	ss2 << ".txt";

	std::string path_bmp;
	ss1 >> path_bmp;

	saveImage(path_bmp);

//	std::string path_data;
//	ss2 >> path_data;
//	std::ofstream myFile(path_data);
//	if (myFile.is_open()) {
//		myFile << "BMP picture file: " << path_bmp << std::endl;
//		myFile << "Image number: " << numImages << std::endl;
//		myFile << "Camera Index: " << cameraIdx << std::endl;
//		myFile << "Camera SN: " << serialNum << std::endl;
//		myFile << "Capture Time CPU: " << captureTimeCPUStr << std::endl;
//		myFile << "Capture Time Cam: " << captureTimeCamStr << std::endl;
//		myFile << "Exposure Time: " << exposureTime << std::endl;
//		myFile << "Gain: " << gain << std::endl;
//		myFile << "Balance Red  : " << balanceR << std::endl;
//		myFile << "Balance Green: " << balanceG << std::endl;
//		myFile << "Balance Blue : " << balanceB << std::endl;
//		myFile << "Auto Exposure Time Continuous: " << autoExpTime << std::endl;
//		myFile << "Auto Gain Continuous: " << autoGain << std::endl;
//		myFile.close();
//	} else {
//		throw std::runtime_error("Could not open the file to save camera data");
//	}
}

void ImagesCV::saveDataConcat (std::string path, Images &img2) {

	// Saves the opencv image and the camera data to file
	// Here path is the path to the directory where the images will be stored.
	// The image number and the camera index are extracted from the object.
	// The function will automatically add the .bmp for the opencv data image and .txt for the camera
	// configuration.

	std::stringstream ss1 { };

	std::string year = captureTimeCPUStr.substr(0, captureTimeCPUStr.find_first_of("-"));
	std::string token = captureTimeCPUStr.substr(captureTimeCPUStr.find_first_of("-")+1, captureTimeCPUStr.length());
	std::string month = token.substr(0, token.find_first_of("-"));
	token = token.substr(token.find_first_of("-")+1, token.length());
	std::string day = token.substr(0, token.find_first_of(" "));
	token = token.substr(token.find_first_of(" ")+1, token.length());
	std::string hour = token.substr(0, token.find_first_of(":"));
	token = token.substr(token.find_first_of(":")+1, token.length());
	std::string min = token.substr(0, token.find_first_of(":"));
	token = token.substr(token.find_first_of(":")+1, token.length());
	std::string sec = token.substr(0, token.find_first_of(":"));
	token = token.substr(token.find_first_of(":")+1, token.length());
	std::string milisec = token.substr(0, token.find_first_of(":"));
	token = token.substr(token.find_first_of(":")+1, token.length());
	std::string microsec = token;

	std::stringstream ss_milisec { };
	ss_milisec << std::setw(3) << std::setfill('0') << std::stoi(milisec);
	ss_milisec >> milisec;

	std::stringstream ss_microsec { };
	ss_microsec << std::setw(3) << std::setfill('0') << std::stoi(microsec);
	ss_microsec >> microsec;

	ss1 << path;
	ss1 << year;
	ss1 << month;
	ss1 << day;
	ss1 << "-";
	ss1 << hour;
	ss1 << min;
	ss1 << sec;
	ss1 << "-";
	ss1 << milisec;
	ss1 << microsec;
	ss1 << ".bmp";

	std::string path_bmp;
	ss1 >> path_bmp;

	cv::Mat m;
	try {
		m = *p_openCvImage;
		cv::hconcat(*p_openCvImage, *((dynamic_cast<ImagesCV &>(img2)).p_openCvImage), m);
	} catch (...) {
		m = *p_openCvImage;
	}

	try {
		imwrite(path_bmp, m);
	} catch (std::exception & ex) {
		std::cerr << "Error writing the bmp file for concatenated ImagesCV objects: " << ex.what() << std::endl;
		throw ex;
	}
}

ImagesCV::~ImagesCV() {
	delete p_openCvImage;
}

} /* namespace ScanVan */
