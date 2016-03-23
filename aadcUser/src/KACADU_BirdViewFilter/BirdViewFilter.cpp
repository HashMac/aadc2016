#include "BirdViewFilter.h"
#include <iostream>

ADTF_PLUGIN_BEGIN(MyPluginClass, "My Plugin description", 0x00)
    ADTF_MAP_FILTER(OID_ADTF_KACADU_BIRDVIEWFILTER, BirdViewFilter)
ADTF_PLUGIN_END(MyPluginClass)

BirdViewFilter::BirdViewFilter(const tChar* __info)
	: InheritThroughput(__info)
	, mCompleteCalFile( "/home/aadc/AADC/src/aadcUser/config/Goffin/BirdviewCal_lowRes.yml" )
{
	SetPropertyStr("Complete Calibration File", mCompleteCalFile.c_str() );	
}

BirdViewFilter::~BirdViewFilter()
{
}

void BirdViewFilter::Startup()
{
	// Set up output path:
	tChar mTmpPath[512];
	GetPropertyStr("Complete Calibration File",
			mTmpPath, 512 );
	mCompleteCalFile.assign( mTmpPath );

    readCompleteCalFile();

	mCoordConverter.readConfigFile( mCompleteCalFile );
}

cv::Mat BirdViewFilter::ProcessImage( cv::Mat &image )
{
    cv::Mat dest;
    cv::Mat view = image.clone();
    cv::undistort(view,dest,mCameraMatrix,mDistCoeffs);
    cv::warpPerspective(dest,view,mWarpMatrix,mImgSize,cv::INTER_LINEAR| cv::WARP_FILL_OUTLIERS, cv::BORDER_CONSTANT);

	return view;
	//return drawReferenceLines( view );
}

cv::Mat BirdViewFilter::drawReferencePoint(cv::Mat img, int x, int y)
{
	cv::Mat dest;
	img.copyTo(dest);
	int thickness = 2;
	int lineType = 8;

	cv::circle( dest,cv::Point2f(x,y),
				5,
				cv::Scalar( 0, 0, 255 ),
				thickness,
				lineType );
	return dest;
}

/*cv::Mat BirdViewFilter::drawReferenceLines(cv::Mat img)
{
	int thickness = 1;
	int lineType = 8;
	
	int xImage = 0;
	int yImage = 0;

	mCoordConverter.getImageGrid( 0.30, 0.50, &xImage, &yImage );
	cv::Point2f start( xImage, yImage );
	mCoordConverter.getImageGrid( -0.30, 0.50, &xImage, &yImage );
	cv::Point2f end( xImage, yImage );

	line( img,
			start,
			end,
			cv::Scalar( 0, 255, 0 ),
			thickness,
			lineType );

	mCoordConverter.getImageGrid( 0.30, 1.00, &xImage, &yImage );
	start = cv::Point2f( xImage, yImage );
	mCoordConverter.getImageGrid( -0.30, 1.00, &xImage, &yImage );
	end = cv::Point2f( xImage, yImage );

	line( img,
			start,
			end,
			cv::Scalar( 0, 128, 0 ),
			thickness,
			lineType );

	mCoordConverter.getImageGrid( 0.30, 1.50, &xImage, &yImage );
	start = cv::Point2f( xImage, yImage );
	mCoordConverter.getImageGrid( -0.30, 1.50, &xImage, &yImage );
	end = cv::Point2f( xImage, yImage );

	line( img,
			start,
			end,
			cv::Scalar( 0, 64, 0 ),
			thickness,
			lineType );

	return img;	
}*/

void BirdViewFilter::readCompleteCalFile() {
	cv::FileStorage fs2(mCompleteCalFile, cv::FileStorage::READ);
	if(fs2.isOpened()){
		std::cout<<"open camera calibration file"<<std::endl;
		cv::FileNode nodeCameraMatrix, nodeDistCoeffs, nodewarpMatrix, nodeRatio,nodeYOffset,nodeScale,nodeCalPoints;

		nodewarpMatrix = fs2["warpMatrix"];
		if(nodewarpMatrix.empty()){
			std::cout<<"can't find node nodewarpMatrix please check the file" << std::endl;
		}
		else{
            fs2["distortion_coefficients"]>>mDistCoeffs;
            fs2["camera_matrix"]>>mCameraMatrix;
			fs2["warpMatrix"]>>mWarpMatrix;
			fs2["ImgSize"]>>mImgSize;
		}
		std::cout << "Read values:" << std::endl;
		std::cout << mDistCoeffs << std::endl;
		std::cout << mCameraMatrix << std::endl;
		std::cout << mWarpMatrix << std::endl;
		std::cout << mImgSize << std::endl;
	}
	else
	{
		std::cout<<"can't open complete Calibration File"<<std::endl;
	}
	fs2.release();
}
