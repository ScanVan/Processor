#ifndef SRC_IMAGES_HPP_
#define SRC_IMAGES_HPP_

#include <vector>
#include <string>
#include <stdint.h>
#include <iostream>
#include <fstream>
#include <chrono>
#include <sstream>

// Include files to use OpenCV API
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace ScanVan {

class Images {
protected:
	size_t height = 3008;
	size_t width = 3008;
	size_t cameraIdx = 0;	// camera index
	std::string captureTimeCPUStr = {}; // capture time taken on the CPU in string format
	std::string captureTimeCamStr = {}; // trigger time retrieved from the camera, number of ticks, string format
	double exposureTime = 0;// exposure time
	int64_t gain = 0;		// gain
	double balanceR = 0;	// white balance R
	double balanceG = 0;	// white balance G
	double balanceB = 0;	// white balance B
	int autoExpTime = 0;	// Auto Exposure Time
	int autoGain = 0; 		// Auto Gain
	long int numImages = 0;
	std::string serialNum{ };
public:
	Images();

	void setHeight(size_t h) {height = h;};
	void setWidth(size_t w) {width = w;};
	void setCameraIdx (size_t idx) { cameraIdx = idx; };
	void setCaptureCPUTime (std::string ct) { captureTimeCPUStr = ct; };
	void setCaptureCamTime (std::string ct) { captureTimeCamStr = ct; };
	void setExposureTime (double et) { exposureTime = et; };
	void setGain (int64_t g) { gain = g; };
	void setBalanceR (double r) { balanceR = r; };
	void setBalanceG (double g) { balanceG = g; };
	void setBalanceB (double b) { balanceB = b; };
	void setAutoExpTime (int b) { autoExpTime = b; };
	void setAutoGain (int b) { autoGain = b; };
	void setSerialNumber (const std::string &sn) { serialNum = sn; };
	void setImgNumber (long int n) { numImages = n; };

	size_t getHeight() const { return height;};
	size_t getWidth() const { return width;};
	size_t getCameraIdx() const { return cameraIdx; };
	std::string getCaptureCPUTime() const { return captureTimeCPUStr; };
	std::string getCaptureCamTime() const { return captureTimeCamStr; };
	double getExposureTime() const { return exposureTime; };
	int64_t getGain() const { return gain; };
	double getBalanceR () const { return balanceR; };
	double getBalanceG () const { return balanceG; };
	double getBalanceB () const { return balanceB; };
	std::string getSerialNumber() const { return serialNum; };
	int getAutoExpTime() const { return autoExpTime; };
	int getAutoGain() const { return autoGain; };
	long int getImgNumber () const { return numImages; };

	virtual void show () const {};
	virtual void show (std::string name) const {};
	virtual void showConcat (std::string name, Images &img2) const {};
	virtual void loadData (std::string path) {};
	virtual void saveData (std::string path) {};
	virtual void loadImage (std::string path) {};
	virtual void saveImage (std::string path) {};
	virtual size_t getImgBufferSize () const { return 0; };

	virtual ~Images();
};

} /* namespace ScanVan */

#endif /* SRC_IMAGES_HPP_ */
