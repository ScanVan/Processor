#ifndef SRC_IMAGESCV_HPP_
#define SRC_IMAGESCV_HPP_

#include "Images.hpp"
#include "ImagesRaw.hpp"

namespace ScanVan {

class ImagesCV: public Images {
private:
	cv::Mat * p_openCvImage;
public:
	ImagesCV();
	ImagesCV(ImagesRaw &img);
	ImagesCV(ImagesCV &img);
	ImagesCV(ImagesCV &&img);

	void show () const;
	void show (std::string name) const;
	void showConcat (std::string name, Images &img2) const;
	void remap (const cv::Mat & map_1, const cv::Mat & map_2);
	void saveImage (std::string path);
	void saveData (std::string path);
	void saveDataConcat (std::string path, Images &img2);

	size_t getImgBufferSize () const { return (*p_openCvImage).total() * (*p_openCvImage).elemSize();};

	virtual ~ImagesCV();
};

} /* namespace ScanVan */

#endif /* SRC_IMAGESCV_HPP_ */
