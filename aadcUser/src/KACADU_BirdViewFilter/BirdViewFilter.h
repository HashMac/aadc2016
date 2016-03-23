#ifndef BIRDVIEWFILTER_H
#define BIRDVIEWFILTER_H

#define OID_ADTF_KACADU_BIRDVIEWFILTER "adtf.aadc.kacadu.birdviewfilter"

#include "BirdViewPosConv.h"
#include "InheritThroughput.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>

//*************************************************************************************************
/*! BirdViewFilter
 *	Filter for Transforming input RGB images from normal camera view to "Bird View". Calibration
 *  currently only for 640x480 images.
 */
class BirdViewFilter : public InheritThroughput
{
    ADTF_FILTER(OID_ADTF_KACADU_BIRDVIEWFILTER, "KACADU Bird View Filter", adtf::OBJCAT_DataFilter);

public:
	//! Constructor
	BirdViewFilter(const tChar* __info);
	//! Destructor
	virtual ~BirdViewFilter();

	void Startup();

	cv::Mat ProcessImage( cv::Mat &image );

private:

    //!Matrix to transform the Image
	cv::Mat mWarpMatrix;
	//! Matrix to disort image
	cv::Mat mCameraMatrix, mDistCoeffs;

	//! size of the picture
	cv::Size mImgSize;

    //! Path to the complete calibration FIle
	std::string mCompleteCalFile;

    //! Read Calibration File with WarpMatrix
	void readCompleteCalFile(void);
	//! Draw reference Point
	cv::Mat drawReferencePoint(cv::Mat img, int x, int y);

	//! Draw reference Lines
	cv::Mat drawReferenceLines(cv::Mat img);

	BirdViewPosConv mCoordConverter;
};

//*************************************************************************************************
#endif
