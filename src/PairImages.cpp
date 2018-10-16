//============================================================================
// Name        : PairImages.cpp
// Author      : Marcelo Kaihara
// Version     : 1.0
// Copyright   :
// Description : It encapsulates images from the two cameras into one entity.
//============================================================================

#include "PairImages.hpp"

namespace ScanVan {

PairImages::PairImages() {
	p_img0 = new Images {};
	p_img1 = new Images {};
}

PairImages::PairImages(Images &a, Images &b) {

	try {
		p_img0 = new ImagesRaw { dynamic_cast<ImagesRaw &>(a) };
	} catch (...) {
	}
	try {
		p_img1 = new ImagesRaw { dynamic_cast<ImagesRaw &>(b) };
	} catch (...) {
	}
	try {
		p_img0 = new ImagesCV { dynamic_cast<ImagesCV &>(a) };
	} catch (...) {
	}
	try {
		p_img1 = new ImagesCV { dynamic_cast<ImagesCV &>(b) };
	} catch (...) {
	}

}

PairImages::PairImages(Images &&a, Images &&b) {

	try {
		p_img0 = new ImagesRaw { std::move(dynamic_cast<ImagesRaw &>(a)) };
	} catch (...) {
	}
	try {
		p_img1 = new ImagesRaw { std::move(dynamic_cast<ImagesRaw &>(b)) };
	} catch (...) {
	}
	try {
		p_img0 = new ImagesCV { std::move(dynamic_cast<ImagesCV &>(a)) };
	} catch (...) {
	}
	try {
		p_img1 = new ImagesCV { std::move(dynamic_cast<ImagesCV &>(b)) };
	} catch (...) {
	}

//	p_img0 = new ImagesRaw { std::move(a) };
//	p_img1 = new ImagesRaw { std::move(b) };
}

PairImages::PairImages(Images &a) {
	try {
		p_img0 = new ImagesRaw { dynamic_cast<ImagesRaw &>(a) };
		p_img1 = new ImagesRaw {};
	} catch (...) {
	}

	try {
		p_img0 = new ImagesCV { dynamic_cast<ImagesCV &>(a) };
		p_img1 = new ImagesCV {};
	} catch (...) {
	}

//	p_img0 = new ImagesRaw { a };
//	p_img1 = new ImagesRaw {};
}

PairImages::PairImages(Images &&a) {
	try {
		p_img0 = new ImagesRaw { std::move(dynamic_cast<ImagesRaw &>(a)) };
		p_img1 = new ImagesRaw { };
	} catch (...) {
	}

	try {
		p_img0 = new ImagesCV { std::move(dynamic_cast<ImagesCV &>(a)) };
		p_img1 = new ImagesCV { };
	} catch (...) {
	}


//	p_img0 = new ImagesRaw { std::move(a) };
//	p_img1 = new ImagesRaw {};
}

PairImages::PairImages(PairImages &a) {

	ImagesRaw *p0{};
	ImagesRaw *p1{};

	if ((p0 = dynamic_cast<ImagesRaw *>(a.p_img0))) {
		p_img0 = new ImagesRaw { *p0 };
	}
	if ((p1 = dynamic_cast<ImagesRaw *>(a.p_img1))) {
		p_img1 = new ImagesRaw { *p1 };
	}

	ImagesCV *p3{};
	ImagesCV *p4{};

	if ((p3 = dynamic_cast<ImagesCV *>(a.p_img0))) {
		p_img0 = new ImagesCV { *p3 };
	}
	if ((p4 = dynamic_cast<ImagesCV *>(a.p_img1))) {
		p_img1 = new ImagesCV { *p4 };
	}

	imgType = a.imgType;

}

PairImages::PairImages(PairImages &&a) {

	/*ImagesRaw *p0{};
	ImagesRaw *p1{};

	if ((p0 = dynamic_cast<ImagesRaw *>(a.p_img0))) {
		p_img0 = new ImagesRaw { *p0 };
	}
	if ((p1 = dynamic_cast<ImagesRaw *>(a.p_img1))) {
		p_img1 = new ImagesRaw { *p1 };
	}

	ImagesCV *p3{};
	ImagesCV *p4{};

	if ((p3 = dynamic_cast<ImagesCV *>(a.p_img0))) {
		p_img0 = new ImagesCV { *p3 };
	}
	if ((p4 = dynamic_cast<ImagesCV *>(a.p_img1))) {
		p_img1 = new ImagesCV { *p4 };
	}*/

	p_img0 = a.p_img0;
	p_img1 = a.p_img1;

	a.p_img0 = nullptr;
	a.p_img1 = nullptr;

	imgType = a.imgType;
}

void PairImages::convertRaw2CV() {

	ImagesRaw *p0 { };
	ImagesRaw *p1 { };

	if (p_img0->getImgBufferSize() != 0)
		if ((p0 = dynamic_cast<ImagesRaw *>(p_img0))) {
			p_img0 = new ImagesCV { *p0 };
			delete p0;
		}
	if (p_img1->getImgBufferSize() != 0)
		if ((p1 = dynamic_cast<ImagesRaw *>(p_img1))) {
			p_img1 = new ImagesCV { *p1 };
			delete p1;
		}

	imgType = ImgType::CV;
}

void PairImages::convertCV2Equi(const cv::Mat & map_0_1, const cv::Mat & map_0_2, const cv::Mat & map_1_1, const cv::Mat & map_1_2) {

	ImagesCV *p0 {};
	ImagesCV *p1 {};

	if (imgType == ImgType::CV) {
		if (p_img0->getImgBufferSize() != 0)
			if ((p0 = dynamic_cast<ImagesCV *>(p_img0))) {
				p0->remap(map_0_1, map_0_2);
			}
		if (p_img1->getImgBufferSize() != 0)
			if ((p1 = dynamic_cast<ImagesCV *>(p_img1))) {
				p1->remap(map_1_1, map_1_2);
			}

		imgType = ImgType::EQUI;
	}

}

void PairImages::showPair() {
	if (p_img0->getImgBufferSize() != 0)
		p_img0->show(p_img0->getSerialNumber());
	if (p_img1->getImgBufferSize() != 0)
		p_img1->show(p_img1->getSerialNumber());
}

void PairImages::showPairConcat() {
	if (p_img1->getImgBufferSize() != 0) {
		p_img0->showConcat(p_img0->getSerialNumber() + "_" + p_img1->getSerialNumber(), *p_img1);
	} else {
		p_img0->show(p_img0->getSerialNumber());
	}
}

/*
void PairImages::showUndistortPairConcat (const cv::Mat & map_0_1, const cv::Mat & map_0_2, const cv::Mat & map_1_1, const cv::Mat & map_1_2) {

	if (p_img1->getImgBufferSize() != 0) {
		cv::Mat m0 = p_img0->convertToCvMat();
		cv::Mat m1 = p_img1->convertToCvMat();

		cv::Mat undistorted_0;
		cv::Mat undistorted_1;

		// main remapping function that undistort the images
		cv::remap(m0, undistorted_0, map_0_1, map_0_2, cv::INTER_CUBIC, cv::BORDER_CONSTANT);
		cv::remap(m1, undistorted_1, map_1_1, map_1_2, cv::INTER_CUBIC, cv::BORDER_CONSTANT);

		cv::Mat m;
		cv::hconcat(undistorted_0, undistorted_1, m);

		/// Display
		cv::namedWindow("Equirectangular_" + p_img0->getSerialNumber() + "_" + p_img1->getSerialNumber(), cv::WINDOW_NORMAL);
		cv::imshow("Equirectangular_" + p_img0->getSerialNumber() + "_" + p_img1->getSerialNumber(), m);
	} else {
		cv::Mat m0 = p_img0->convertToCvMat();

		cv::Mat undistorted_0;

		// main remapping function that undistort the images
		cv::remap(m0, undistorted_0, map_0_1, map_0_2, cv::INTER_CUBIC, cv::BORDER_CONSTANT);

		/// Display
		cv::namedWindow("Equirectangular_" + p_img0->getSerialNumber(),cv::WINDOW_NORMAL);
		cv::imshow("Equirectangular_" + p_img0->getSerialNumber(), undistorted_0);
	}
}
*/

void PairImages::savePair(std::string path) {
	if ((imgType == ImgType::RAW) || (imgType == ImgType::CV)) {
		if (p_img0->getImgBufferSize() != 0) {
			p_img0->saveData(path);
		}
		if (p_img1->getImgBufferSize() != 0) {
			p_img1->saveData(path);
		}
	} else if (imgType == ImgType::EQUI) {
		(dynamic_cast<ImagesCV *>(p_img0))->saveDataConcat(path, *p_img1);
	}
}

void PairImages::setImgNumber (const long int &n) {
	/*p_img0->setImgNumber(n);
	p_img1->setImgNumber(n);*/
	imgNum = n;
}

PairImages & PairImages::operator=(const PairImages &a){
	if (this != &a) {

		delete p_img0;
		delete p_img1;

		ImagesRaw *p0 { };
		ImagesRaw *p1 { };

		if ((p0 = dynamic_cast<ImagesRaw *>(a.p_img0))) {
			p_img0 = new ImagesRaw { *p0 };
		}
		if ((p1 = dynamic_cast<ImagesRaw *>(a.p_img1))) {
			p_img1 = new ImagesRaw { *p1 };
		}

		ImagesCV *p3 { };
		ImagesCV *p4 { };

		if ((p3 = dynamic_cast<ImagesCV *>(a.p_img0))) {
			p_img0 = new ImagesCV { *p3 };
		}
		if ((p4 = dynamic_cast<ImagesCV *>(a.p_img1))) {
			p_img1 = new ImagesCV { *p4 };
		}

	}
	return *this;
}

PairImages & PairImages::operator=(PairImages &&a){
	if (this != &a) {
		delete p_img0;
		delete p_img1;
		p_img0 = a.p_img0;
		p_img1 = a.p_img1;
		a.p_img0 = nullptr;
		a.p_img1 = nullptr;
	}
	return *this;
}


PairImages::~PairImages() {
	delete p_img0;
	delete p_img1;
}

} /* namespace ScanVan */
