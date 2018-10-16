//============================================================================
// Name        : Images.hpp
// Author      : Marcelo Kaihara
// Version     : 1.0
// Copyright   :
// Description : It has the routines for manimulating the underlying images.
// 				 It has functions to write and load images together with camera configuration
//				 It has functions that displays the images on screen.
//============================================================================

#ifndef IMAGESRAW_HPP_
#define IMAGESRAW_HPP_

#include <vector>
#include <string>
#include <stdint.h>
#include <iostream>
#include <fstream>
#include <chrono>
#include <sstream>

#include "Images.hpp"

// Include files to use OpenCV API
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace ScanVan {

class ImagesRaw: public Images {
private:
	std::vector<uint8_t> * p_img;

protected:
	std::string convertTimeToString (time_t t);
	time_t convertStringToTime (std::string str);

public:
	ImagesRaw();
	ImagesRaw(char * p);
	ImagesRaw(size_t h, size_t w);
	ImagesRaw(size_t h, size_t w, char * p);
	ImagesRaw(std::string path);
	ImagesRaw(const ImagesRaw &img);
	ImagesRaw(ImagesRaw &&img);

	size_t getImgBufferSize () const { return p_img->size(); };

	void getBuffer (char *p) const;
	void copyBuffer (char *p);
	uint8_t* getBufferP ();

	void loadImage (std::string path);
	void saveImage (std::string path);
	void loadData (std::string path);
	void saveData (std::string path);
	void show () const;
	void show (std::string name) const;
	void showConcat (std::string name, Images &img2) const;

	cv::Mat convertToCvMat ();

	ImagesRaw & operator=(const ImagesRaw &a);
	ImagesRaw & operator=(ImagesRaw &&a);

	virtual ~ImagesRaw();

	friend std::ostream & operator <<(std::ostream & out, const ImagesRaw &a) {
		out << "[";
		for (int i=0; i < 10; ++i) {
			out << static_cast<int>((*a.p_img)[i]) << " ";
		}
		out << "...]" << std::endl;
		out << "height: " << a.height << std::endl;
		out << "width: " << a.width << std::endl;
		out << "cameraIdx: " << a.cameraIdx << std::endl;
		out << "captureCPUTime: " << a.captureTimeCPUStr << std::endl;
		out << "captureCamTime: " << a.captureTimeCamStr << std::endl;
		out << "exposureTime: " << a.exposureTime << std::endl;
		out << "gain: " << a.gain << std::endl;
		out << "balanceR: " << a.balanceR << std::endl;
		out << "balanceG: " << a.balanceG << std::endl;
		out << "balanceB: " << a.balanceB << std::endl;
		out << "autoExpTime: " << a.autoExpTime << std::endl;
		out << "autoGain: " << a.autoGain << std::endl;

		return out;
	}
};

} /* namespace ScanVan */

#endif /* IMAGESRAW_HPP_ */
