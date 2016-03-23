
#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>
using namespace adtf;

#include "BirdViewCalibration.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>

#include <iostream>

#include "adtf2OpenCV.h"

ADTF_FILTER_PLUGIN("Bird View Calibration Filter", OID_ADTF_KACADU_BIRDVIEWCALIBRATION, BirdViewCalibration);

BirdViewCalibration::BirdViewCalibration(const tChar* __info):QObject(),cBaseQtFilter(__info)
{
	// Copy 0 into all the bytes of mInputFormat and mOutputFormat:
	cMemoryBlock::MemSet(&mInputFormat, 0, sizeof(mInputFormat));
	cMemoryBlock::MemSet(&mOutputFormat, 0, sizeof(mOutputFormat));


	mRatio = 1;
	mYOffset = 400;
	mScale = 1;
	mImgSize = cv::Size(640	,480);
	mPointsHeight = 5;
	mPointsWidth = 8;
	//mRatio = ((double)mPointsHeight-1.0)/((double)mPointsWidth-1.0);
	mRatio = 0.57286f;    // Measured ratio of calibration pattern
	std::cout<<"[BirdViewCalibration] mRatio"<<mRatio<<std::endl;
	mDistance2PointM = 0.05;      // Distance between two points in calibration pattern
	//mDistanceCarToCalPattern = 0.276;
	mDistanceCarToCalPattern = 0.276 + 0.502;
    mNumberOfFounds = 0;
	mCameraCalFile = "/home/aadc/AADC/src/aadcUser/config/Goffin/xtion_intrinsic_calib_low_res.yml";
	mCompleteCalFile = "/home/aadc/AADC/src/aadcUser/config/Goffin/BirdviewCal_lowRes.yml";
	mCalibrationFinished = false;
	readCameraCalFile();
}

BirdViewCalibration::~BirdViewCalibration()
{

}

tResult BirdViewCalibration::Init(tInitStage eStage, __exception)
{
	// never miss calling the parent implementation!!
	RETURN_IF_FAILED(cBaseQtFilter::Init(eStage, __exception_ptr));

		// in StageFirst you can create and register your static pins.
		if (eStage == StageFirst)
		{
			// create and register the input pin
			RETURN_IF_FAILED(mPinVideoInput.Create("video_input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
			RETURN_IF_FAILED(RegisterPin(&mPinVideoInput));
			RETURN_IF_FAILED(mPinVideoOutput.Create("video_output", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
			RETURN_IF_FAILED(RegisterPin(&mPinVideoOutput));

		}
		else if (eStage == StageNormal)
		{
			// In this stage you would do further initialisation and/or create your dynamic pins.
			// Please take a look at the demo_dynamicpin example for further reference.
		}
		else if (eStage == StageGraphReady)
		{
			// All pin connections have been established in this stage so you can query your pins
			// about their media types and additional meta data.
			// Please take a look at the demo_imageproc example for further reference.

			// Get the image format of the input pin (which is determined by the filter before it)
			cObjectPtr<IMediaType> pType;
			RETURN_IF_FAILED(mPinVideoInput.GetMediaType(&pType));

			cObjectPtr<IMediaTypeVideo> pTypeVideo;
			RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));
			//mInputFormat = *(pTypeVideo->GetFormat());
			//mOutputFormat = *(pTypeVideo->GetFormat());

			const tBitmapFormat* pFormat = pTypeVideo->GetFormat();
			if (pFormat == NULL)
			{
				LOG_ERROR("No Bitmap information found on pin \"input\"");
				RETURN_ERROR(ERR_NOT_SUPPORTED);
			}

			// I don't know what these two lines do yet. They copy pFormat into mInputFormat - but why??
			mInputFormat = *pFormat;
			mOutputFormat = *pFormat;
			//cMemoryBlock::MemCopy(&mInputFormat, pFormat, sizeof(tBitmapFormat));
			//cMemoryBlock::MemCopy(&mOutputFormat, pFormat, sizeof(tBitmapFormat));

			// Set the format of the output pin to the same format as the input pin!
			mPinVideoOutput.SetFormat(&mOutputFormat, NULL);
		}

	RETURN_NOERROR;
}

tResult BirdViewCalibration::Shutdown(tInitStage eStage, __exception)
{
	// In each stage clean up everything that you initiaized in the corresponding stage during Init.
	// Pins are an exception:
	// - The base class takes care of static pins that are members of this class.
	// - Dynamic pins have to be cleaned up in the ReleasePins method, please see the demo_dynamicpin
	//   example for further reference.

	if (eStage == StageGraphReady)
	{
	}
	else if (eStage == StageNormal)
	{
	}
	else if (eStage == StageFirst)
	{
	}

	// call the base class implementation
	 return cBaseQtFilter::Shutdown(eStage, __exception_ptr);
}

tResult BirdViewCalibration::OnPinEvent(IPin* pSource,
		tInt nEventCode,
		tInt nParam1,
		tInt nParam2,
		IMediaSample* pMediaSample)
{
	// first check what kind of event it is
	if (pSource == &mPinVideoInput && nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{
		// so we received a media sample, so this pointer better be valid.
		RETURN_IF_POINTER_NULL(pMediaSample);

		// Read image:
		const tVoid* l_pSrcBuffer;
		cImage iImage;
		adtf::cScopedSampleReadLock rl1( pMediaSample, &l_pSrcBuffer );
		RETURN_IF_FAILED(iImage.Attach((tUInt8*)l_pSrcBuffer, &mInputFormat, NULL));

		// Convert to CV image:
		cv::Mat image;
		image = cImageToCV(iImage);

		// Do the actual transformation:
		cv::Mat transformed = ProcessImage( image );

		// Convert back to ADTF image:
		iImage = cvToCImage( transformed );

		// Ready media type:
		cObjectPtr<IMediaSample> pNewSample;
		RETURN_IF_FAILED(_runtime->CreateInstance(OID_ADTF_MEDIA_SAMPLE, IID_ADTF_MEDIA_SAMPLE, (tVoid**) &pNewSample));
		RETURN_IF_FAILED(pNewSample->AllocBuffer(mOutputFormat.nSize));
		tVoid* l_pDestBuffer;
		if (IS_OK(pNewSample->WriteLock(&l_pDestBuffer)))
		{
			if (mOutputFormat.nBitsPerPixel == mInputFormat.nBitsPerPixel )
			{
				cImage::StretchBlit((tUInt8*) iImage.GetBitmap(),
						&mInputFormat,
						(tUInt8*) l_pDestBuffer,
						&mOutputFormat,
						0);
			}
			pNewSample->Unlock( l_pDestBuffer );
		}


		pNewSample->SetTime(pMediaSample->GetTime());
		mPinVideoOutput.Transmit(pNewSample);

	}
	else if (pSource == &mPinVideoInput && nEventCode == IPinEventSink::PE_MediaTypeChanged)
	{
		cObjectPtr<IMediaType> pType;
		RETURN_IF_FAILED(mPinVideoInput.GetMediaType(&pType));

		cObjectPtr<IMediaTypeVideo> pTypeVideo;
		RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));

		const tBitmapFormat* pFormat = pTypeVideo->GetFormat();
		if (pFormat != NULL)
		{
		    cMemoryBlock::MemCopy(&mInputFormat, pFormat, sizeof(tBitmapFormat));
		    cMemoryBlock::MemCopy(&mOutputFormat, pFormat, sizeof(tBitmapFormat));
		    mPinVideoOutput.SetFormat(&mOutputFormat, NULL);
		}
	}
	RETURN_NOERROR;
}

cv::Mat BirdViewCalibration::ProcessImage( cv::Mat image )
{
	bool found;
	cv::Mat view;

	if(m_calibrationState == CAPTURING ){

        found = autoCalPoints(image);
        if(found == true)
        {
            mNumberOfFounds++;
            if(mNumberOfFounds == 8)
            {
                //now we have hopefully a stabel Image
                m_calibrationState = CALIBRATED;
                emit sendState(CALIBRATED);


            }
            std::cout<<"pattern found"<<mNumberOfFounds<<"times"<<std::endl;
        }
        view = drawCalPoints(image);
    }
    else if(m_calibrationState == CALIBRATED)
    {
        view = transform(image);
std::cout<<"calibriert"<<std::endl;
    }
    else
    {
        view = image;
    }

	return view;
}


void  BirdViewCalibration::calculateWrapMatrix()
{
    std::cout<<"[BirdViewCalibration] mRatio"<<mRatio<<std::endl;
	std::cout<<"[BirdViewCalibration]source Points"<<std::endl;
	std::cout<<mCalPoints[0]<<std::endl;
	std::cout<<mCalPoints[1]<<std::endl;
	std::cout<<mCalPoints[2]<<std::endl;
	std::cout<<mCalPoints[3]<<std::endl;


	//calculate destination pixels. We suppose following pattern:
	/*				x
	 * Point0			Point1
	 *
	 *Y
	 *
	 * Point2			Point3
	 *
	 * in the real world the points must be quadratic and parallel to the car
	 */



	int distance2Points;
	distance2Points = cv::norm(mCalPoints[2]-mCalPoints[3]);
	distance2Points = distance2Points*mScale;
	//Point 2 will be distance2Points/2 left of the center
	mDestVertices[2] = cv::Point(mImgSize.width/2-(distance2Points/2),mYOffset);
	//Point 3 will have the same y coordinates as point 2 and the same x coordinates as in the src img
	mDestVertices[3] = cv::Point(mImgSize.width/2+(distance2Points/2),mDestVertices[2].y);
	//Point 1 has the same x coordinates as point 3 and the same y distance as the x distance of Point 2 and 3
	mDestVertices[1] = cv::Point(mDestVertices[3].x,mDestVertices[3].y-distance2Points*mRatio);
	//Point 0 has the same x coordinates as point 2 and the same y distance as the x distance of Point 2 and 3
	mDestVertices[0] = cv::Point(mDestVertices[2].x,mDestVertices[2].y-distance2Points*mRatio);


	std::cout<<"[BirdViewCalibration]destination Points"<<std::endl;
	std::cout<<mDestVertices[0]<<std::endl;
	std::cout<<mDestVertices[1]<<std::endl;
	std::cout<<mDestVertices[2]<<std::endl;
	std::cout<<mDestVertices[3]<<std::endl;

	mWrapMatrix = cv::getPerspectiveTransform(mCalPoints, mDestVertices);
	//also recalculate the position of birdview
	calcPosBirdviewInCarGrid();



}

void BirdViewCalibration::readCameraCalFile ()
{
	//cv::FileStorage fs2("../../Testbilder/xtion_intrinsic_calib_High_res.yml", cv::FileStorage::READ);
	cv::FileStorage fs2(mCameraCalFile, cv::FileStorage::READ);
	if(fs2.isOpened()){
		std::cout<<"open camera calibration file"<<std::endl;
		cv::FileNode nodeCameraMatrix, nodeDistCoeffs;

		nodeCameraMatrix = fs2["camera_matrix"];
		nodeDistCoeffs = fs2["distortion_coefficients"];
		if(nodeCameraMatrix.empty()|| nodeDistCoeffs.empty()){
			std::cout<<"can't find node camera_matrix or distortion_coefficients please check the file";
		}
		else{
			fs2["camera_matrix"] >> mCameraMatrix;
			fs2["distortion_coefficients"] >> mDistCoeffs;
		}
	}
	else
	{
		std::cout<<"can't open camera calibration File"<<std::endl;
	}
	fs2.release();


}

/*void BirdViewCalibration::readCompleteCalFile() {
	//cv::FileStorage fs2("../../Testbilder/xtion_intrinsic_calib_High_res.yml", cv::FileStorage::READ);
	cv::FileStorage fs2(mCompleteCalFile, cv::FileStorage::READ);
	if(fs2.isOpened()){
		std::cout<<"open camera calibration file"<<std::endl;
		cv::FileNode nodeCameraMatrix, nodeDistCoeffs, nodewarpMatrix, nodeRatio,nodeYOffset,nodeScale,nodeCalPoints;

		nodeCameraMatrix = fs2["camera_matrix"];
		nodeDistCoeffs = fs2["distortion_coefficients"];
		nodewarpMatrix = fs2["warpMatrix"];
		nodeRatio = fs2["Ratio"];
		nodeYOffset= fs2["yOffset"];
		nodeScale = fs2["scale"];
		nodeCalPoints = fs2["CalPoints0"];
		if(nodeCameraMatrix.empty()|| nodeDistCoeffs.empty()||nodewarpMatrix.empty()||nodeRatio.empty()||nodeYOffset.empty()||nodeScale.empty()||nodeCalPoints.empty()){
			std::cout<<"can't find node camera_matrix or distortion_coefficients  nodewarpMatrix, nodeRatio,nodeYOffset,nodeScale,nodeCalPoints please check the file";
		}
		else{
			fs2["camera_matrix"] >> mCameraMatrix;
			fs2["distortion_coefficients"] >> mDistCoeffs;
			fs2["warpMatrix"]>>mWrapMatrix;
			fs2["Ratio"]>>mRatio;
			fs2["yOffset"]>>mYOffset;
			fs2["scale"]>>mScale;
			fs2["CalPoints0"]>>mCalPoints[0];
			fs2["CalPoints1"]>>mCalPoints[1];
			fs2["CalPoints2"]>>mCalPoints[2];
			fs2["CalPoints3"]>>mCalPoints[3];
			fs2["Distance2PointM"]>>mDistance2PointM;
			fs2["DistanceCarToCalPattern"]>>mDistanceCarToCalPattern;
			fs2["CoordinatesBirdView"]>>mCoordinatesBirdView;
			fs2["MPerPixel"]>>mMPerPixel;
			fs2["PointsHeight"]>>mPointsHeight;
			fs2["PointsWidth"]>>mPointsWidth;
			fs2["ImgSize"]>>mImgSize;
		}
	}
	else
	{
		std::cout<<"can't open complete Calibration File"<<std::endl;
	}
	fs2.release();
}*/

void BirdViewCalibration::writeCalFile (const string& path)
{
	cv::FileStorage fs(path, cv::FileStorage::WRITE);
	fs << "camera_matrix" << mCameraMatrix << "distortion_coefficients" << mDistCoeffs<<"warpMatrix"<<mWrapMatrix<<"Ratio"
			<<mRatio<<"yOffset"<<mYOffset<<"scale"<<mScale<<"CalPoints0"<<mCalPoints[0]<<"CalPoints1"<<mCalPoints[1]
			<<"CalPoints2"<<mCalPoints[2]<<"CalPoints3"<<mCalPoints[3]<<"Distance2PointM"<<mDistance2PointM
			<<"DistanceCarToCalPattern"<<mDistanceCarToCalPattern<<"CoordinatesBirdView"<<mCoordinatesBirdView
			<<"MPerPixel"<<mMPerPixel<<"PointsHeight"<<mPointsHeight<<"PointsWidth"<<mPointsWidth<<"ImgSize"<<mImgSize;
    std::cout<<"saved complete calibration file"<<std::endl;

}

cv::Mat BirdViewCalibration::drawCalPoints(cv::Mat img)
{
	cv::Mat dest;
	img.copyTo(dest);
	int thickness = 2;
	int lineType = 8;

	for (int i= 0; i < 4; i++) {
		cv::circle( dest,
				mCalPoints[i],
				5,
				cv::Scalar( 0, 0, 255 ),
				thickness,
				lineType );
		//put the number to the points
		std::string PointNumberString;
		PointNumberString = patch::to_string(i);
		cv::putText(dest, PointNumberString , mCalPoints[i], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,200,200), 4);

	}
	return dest;
}

bool BirdViewCalibration::autoCalPoints(cv::Mat img){

	bool found;
	cv::Mat view;
	std::vector<cv::Point2f> pointBuf;
	cv::Size patternsize(mPointsWidth,mPointsHeight);
	//first distort Image
	img = unDisortImg(img);
	found = cv::findCirclesGrid( img, patternsize, pointBuf, cv::CALIB_CB_SYMMETRIC_GRID| cv::CALIB_CB_CLUSTERING);

	//only go go forward if we have found the pattern
	if(found){
		/*manchmal macht opencv ein paar knoten Die Punkte aus der Kalibrierung sind wie folgt:
		 * Point 3              Point 2
		 *
		 *
		 * Point 1				Point 0
		 *
		 * Sie werden so abgespeichert
		 *
		 * Point 0				Point 1
		 *
		 *
		 * Point 2				Point 3
		 */
		mCalPoints[3] = pointBuf[0];
		mCalPoints[2] = pointBuf[mPointsWidth-1];
		mCalPoints[1] = pointBuf[mPointsWidth*(mPointsHeight-1)];
		mCalPoints[0] = pointBuf[mPointsWidth*mPointsHeight -1];
		calculateWrapMatrix();

	}


	return found;

}

cv::Mat BirdViewCalibration::unDisortImg(cv::Mat img)
{
	cv::Mat dest;
	cv::undistort(img,dest,mCameraMatrix,mDistCoeffs);
	return dest;


}

void BirdViewCalibration::calcPosBirdviewInCarGrid() {
	double distance2PointsPx = mDestVertices[3].x-mDestVertices[2].x;//the vertices are the 2 outer Points so there are N Points between them
	mMPerPixel = (mDistance2PointM*(mPointsWidth-1))/distance2PointsPx;
	mCoordinatesBirdView.y = mDistanceCarToCalPattern-(mImgSize.height-mDestVertices[2].y)*mMPerPixel;
	mCoordinatesBirdView.x = ((-1)*(mDestVertices[2].x+(distance2PointsPx/2)))*mMPerPixel;



}

cv::Mat BirdViewCalibration::transform(cv::Mat image)
{
	cv::Mat dest;
	cv::Mat temp;
	temp = unDisortImg(image);
	cv::warpPerspective(image,dest,mWrapMatrix,mImgSize,cv::INTER_LINEAR| cv::WARP_FILL_OUTLIERS, cv::BORDER_CONSTANT);
	return dest;


}

tHandle BirdViewCalibration::CreateView()
{
    QWidget* pWidget = (QWidget*)m_pViewport->VP_GetWindow();
    m_pWidget = new DisplayWidget(pWidget);

    //doing the connection between gui and filter
    connect(m_pWidget->m_btStart,SIGNAL(clicked()),this,SLOT(OnStartCalibration()));
    connect(m_pWidget,SIGNAL(SendSaveAs(QString)),this,SLOT(OnSaveAs(QString)));
    connect(this,SIGNAL(sendState(int)),m_pWidget,SLOT(OnSetState(int)));
    connect(m_pWidget->mYOffsetSlider, SIGNAL(valueChanged(int)), this, SLOT(OnYOffset(int)));
    connect(m_pWidget->mScaleSlider, SIGNAL(valueChanged(int)), this, SLOT(OnScale(int)));

    return (tHandle)m_pWidget;
}

tResult  BirdViewCalibration::ReleaseView()
{
    if (m_pWidget != NULL)
    {
        delete m_pWidget;
        m_pWidget = NULL;
    }
    RETURN_NOERROR;
}

void BirdViewCalibration::OnStartCalibration()
{
    // switch state to CAPTURING
    m_calibrationState = CAPTURING;
    emit sendState(CAPTURING);
    LOG_INFO("Starting Calibration");
}

void BirdViewCalibration::OnSaveAs(QString qFilename)
{
    // save the calibration results to the given file in argument
    if (m_calibrationState  == CALIBRATED)
        {
          // convert file name to absolute file
        cFilename filename = qFilename.toStdString().c_str();
        ADTF_GET_CONFIG_FILENAME(filename);
        filename = filename.CreateAbsolutePath(".");

        if( true)
            {
            LOG_INFO(cString::Format("Saved calibration to %s",filename.GetPtr()));
            writeCalFile(filename.GetPtr());
            std::cout<<cString::Format("Saved calibration to %s",filename.GetPtr());
            // switch to state WAITING again
            m_calibrationState = WAITING;
            }
        }
}

void BirdViewCalibration::OnYOffset(int YOffset)
{
    mYOffset = YOffset;
    calculateWrapMatrix();

}

void BirdViewCalibration::OnScale(int scale)
{
    mScale = (double)scale/100;
    calculateWrapMatrix();
}


