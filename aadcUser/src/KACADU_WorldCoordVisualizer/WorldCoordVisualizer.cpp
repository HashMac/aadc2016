
#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>
using namespace adtf;

#include "WorldCoordVisualizer.h"

#include "adtf2OpenCV.h"

/// Create filter shell
ADTF_FILTER_PLUGIN("KACADU World Coord Visualizer", OID_ADTF_KACADU_WORLDCOORDVISUALIZER, WorldCoordVisualizer);

using namespace oadrive::core;

WorldCoordVisualizer::WorldCoordVisualizer(const tChar* __info):cFilter(__info)
	, mCarPose(0,0,0)
	, mArduinoTimestamp(0)
	, mLastTime(0)
	, mLastSendTime(0)
	, mTimeBetweenUpdates(0.05e6)	// in ms
	, mOutputWidth(500)
	, mOutputHeight(500)
	, mImageCenter(mOutputWidth*0.5)
	, mMetersToPixels(50.0f)
	, mImageGrid( mOutputWidth, mOutputHeight, CV_8UC4, cv::Scalar(0,0,0,0) )		// RGBA, 8 bit per channel
	, mImagePath( mOutputWidth, mOutputHeight, CV_8UC4, cv::Scalar(0,0,0,0) )		// RGBA, 8 bit per channel
	, mImageOverlay( mOutputWidth, mOutputHeight, CV_8UC4, cv::Scalar(0,0,0,0) )		// RGBA, 8 bit per channel
	, mImage( mOutputWidth, mOutputHeight, CV_8UC4, cv::Scalar(0,0,0,0) )		// RGBA, 8 bit per channel
	, mImageSend( mOutputWidth, mOutputHeight, CV_8UC3, CV_RGB(0,0,0) )		// RGB, 8 bit per channel
	, mImageFlipped( mOutputWidth, mOutputHeight, CV_8UC3, CV_RGB(0,0,0) )		// RGB, 8 bit per channel
	, mPreviousPos( mImageCenter, mImageCenter )
{
	/*mOutputFormat.nWidth = mOutputWidth;
	mOutputFormat.nHeight = mOutputHeight;
	mOutputFormat.nBitsPerPixel = 24*8;
	mOutputFormat.nPixelFormat = IImage::PF_RGB_888;
	mOutputFormat.nWidth * nBitsPerPixel / 8*/
	mADTFImage.Create( mOutputWidth, mOutputHeight, 3*8, 0, NULL, IImage::PF_RGB_888 );
	//Create (tInt nWidth, tInt nHeight, tInt nBitsPerPixel, tInt nBytesPerLine=0, const tUInt8 *pBitmap=NULL, tInt nPixelFormat=0)
	mOutputFormat = *mADTFImage.GetFormat();

	// Draw grid on image:
	int gridLines = (float)mOutputWidth/(float)mMetersToPixels;
	int start = -gridLines*0.5;
	for( int x = start; x <= gridLines - start; x ++ )
	{
		cv::Point2f start( round(x*mMetersToPixels) + mImageCenter, 0 );
		cv::Point2f end( round(x*mMetersToPixels) + mImageCenter, mOutputHeight );
		cv::line( mImageGrid, start, end, cv::Scalar(30, 30, 30, 255) );
	}
	for( int y = start; y <= gridLines - start; y ++ )
	{
		cv::Point2f start( 0, round(y*mMetersToPixels) + mImageCenter );
		cv::Point2f end( mOutputWidth, round(y*mMetersToPixels) + mImageCenter );
		cv::line( mImageGrid, start, end, cv::Scalar(30, 30, 30, 255) );
	}
	// Draw x axis:
	cv::Point2f s( mImageCenter, mImageCenter );
	cv::Point2f e( mOutputWidth*0.5 + mImageCenter, mImageCenter );
	cv::line( mImageGrid, s, e, cv::Scalar(64, 128, 64 ) );
	cv::Point2f textPos( mOutputWidth - 20, mImageCenter - 10 );
	putText( mImageGrid, "x", textPos, cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(64,128,64) );
	// Draw y axis:
	e = cv::Point2f( mImageCenter, 0 );
	textPos = cv::Point2f( mImageCenter + 7, 10 );
	cv::line( mImageGrid, s, e, cv::Scalar(64, 64, 128 ) );
	putText( mImageGrid, "y", textPos, cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(64,64,128) );

	// flip grid image:
	cv::Mat tmp( mOutputWidth, mOutputHeight, CV_8UC4, cv::Scalar(0,0,0,0) );		// RGBA, 8 bit per channel
	cv::flip( mImageGrid, tmp, 0 );
	mImageGrid = tmp;
}

WorldCoordVisualizer::~WorldCoordVisualizer()
{
}

tResult WorldCoordVisualizer::Init(tInitStage eStage, __exception)
{
	// never miss calling the parent implementation!!
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

		// in StageFirst you can create and register your static pins.
		if (eStage == StageFirst)
		{

			//get the description manager for this filter
			cObjectPtr<IMediaDescriptionManager> pDescManager;
			RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

			//input
			//get description for sensor data pins
			tChar const * strInPositionDescription = pDescManager->GetMediaDescription("tPositionFloat");
			RETURN_IF_POINTER_NULL(strInPositionDescription);
			//get mediatype for output position data pin:
			cObjectPtr<IMediaType> pInPositionMedia = new cMediaType(0, 0, 0, "tPositionFloat", strInPositionDescription, IMediaDescription::MDF_DDL_DEFAULT_VERSION); 
			RETURN_IF_FAILED(pInPositionMedia->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&mPositionDescription));

			RETURN_IF_FAILED(mPinPositionInput.Create("car_pos", pInPositionMedia, static_cast<IPinEventSink*>(this)));
			RETURN_IF_FAILED(RegisterPin(&mPinPositionInput));

			tChar const * strInDrawPositionDescription = pDescManager->GetMediaDescription("tDrawPositionFloat");
			RETURN_IF_POINTER_NULL(strInDrawPositionDescription);
			//get mediatype for output position data pin:
			cObjectPtr<IMediaType> pInDrawPositionMedia = new cMediaType(0, 0, 0, "tDrawPositionFloat", strInDrawPositionDescription, IMediaDescription::MDF_DDL_DEFAULT_VERSION); 
			RETURN_IF_FAILED(pInDrawPositionMedia->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&mDrawPositionDescription));

			RETURN_IF_FAILED(mPinDrawPositionInput.Create("draw_point", pInDrawPositionMedia, static_cast<IPinEventSink*>(this)));
			RETURN_IF_FAILED(RegisterPin(&mPinDrawPositionInput));

			//Output (video):

			RETURN_IF_FAILED(mPinVideoOutput.Create("video_output", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
			RETURN_IF_FAILED(RegisterPin(&mPinVideoOutput));
			mPinVideoOutput.SetFormat(&mOutputFormat, NULL);

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
			mPinVideoOutput.SetFormat(&mOutputFormat, NULL);
			//mIDsSignalOutputSet = false;
		}

	RETURN_NOERROR;
}

tResult WorldCoordVisualizer::Shutdown(tInitStage eStage, __exception)
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
	return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult WorldCoordVisualizer::OnPinEvent(IPin* pSource,
		tInt nEventCode,
		tInt nParam1,
		tInt nParam2,
		IMediaSample* pMediaSample)
{
	// first check what kind of event it is
	if (pSource == &mPinPositionInput && nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{
		// so we received a media sample, so this pointer better be valid.
		RETURN_IF_POINTER_NULL(pMediaSample);

		tFloat32 x = 0;
		tFloat32 y = 0;
		tFloat32 yaw = 0;
		tUInt32 timeStamp = 0;

		{// focus for sample read lock

			 // read-out the incoming Media Sample
			__adtf_sample_read_lock_mediadescription(mPositionDescription, pMediaSample, coder);

			//get values from media sample        
			coder->Get("fX", (tVoid*)&x);
			coder->Get("fY", (tVoid*)&y);
			coder->Get("fYaw", (tVoid*)&yaw);
			coder->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
		}

		//std::cout << "time: " << timeStamp << " pos: (" << x << "," << y << ") angle: " << yaw << std::endl;

		mCarPose.setX( x );
		mCarPose.setY( y );
		mCarPose.setYaw( yaw );
		
		mArduinoTimestamp = timeStamp;
		mLastTime = pMediaSample->GetTime();
		
		if( mLastTime - mLastSendTime > mTimeBetweenUpdates )
		{
			updateImage();
			composeImage();
			sendImage();
			mLastSendTime = mLastTime;
		}
	}
	else if (pSource == &mPinDrawPositionInput && nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{
		// so we received a media sample, so this pointer better be valid.
		RETURN_IF_POINTER_NULL(pMediaSample);

		tFloat32 x = 0;
		tFloat32 y = 0;
		tFloat32 seconds = 0;
		tUInt8 red = 0;
		tUInt8 green = 0;
		tUInt8 blue = 0;

		{// focus for sample read lock

			 // read-out the incoming Media Sample
			__adtf_sample_read_lock_mediadescription(mDrawPositionDescription, pMediaSample, coder);

			//get values from media sample        
			coder->Get("fX", (tVoid*)&x);
			coder->Get("fY", (tVoid*)&y);
			coder->Get("fSeconds", (tVoid*)&seconds);
			coder->Get("ui32Red", (tVoid*)&red);
			coder->Get("ui32Green", (tVoid*)&green);
			coder->Get("ui32Blue", (tVoid*)&blue);
		}

		//tTimeStamp currentTime = pMediaSample->GetTime();

		float posX = x*mMetersToPixels + mImageCenter;
		float posY = y*mMetersToPixels + mImageCenter;
		
		//std::cout << "Received point @ " << mLastTime << " " << seconds << std::endl;

		DebugPoint dp;
		dp.pos = cv::Point2f( round( posX ), round( posY ) );
		dp.color = cv::Scalar( blue, green, red, 255 );
		// Calculate the time when the point should no longer be visible
		dp.killTime = mLastTime + seconds*1e6;
		dp.startTime = mLastTime;
		//std::cout << "start: " << dp.startTime << " end: " << dp.killTime << std::endl;
		mDrawPoints.push_back(dp);
	}

	RETURN_NOERROR;
}

void WorldCoordVisualizer::updateImage()
{
	cv::Point2f newPos( round( mCarPose.getX()*mMetersToPixels + mImageCenter ),
						round( mCarPose.getY()*mMetersToPixels + mImageCenter ) );
	cv::line( mImagePath, mPreviousPos, newPos, cv::Scalar( 255, 255, 255, 255 ) );

	// Reset overlay:
	mImageOverlay = cv::Scalar( 0, 0, 0, 0 );
	// Draw debug points:
	std::vector<DebugPoint>::iterator it;
	int i = 0;
	for( it = mDrawPoints.begin(); it != mDrawPoints.end(); )
	{
		cv::circle( mImageOverlay, it->pos, 1, it->color );
		if( it->killTime > mLastTime )
		{
			/*double numer = (double(mLastTime) - double(it->startTime));
			double denom = (double(it->killTime) - double(it->startTime));
			double amount = std::min(std::max(numer/denom,0.0),1.0);
			//double amount = (double(mLastTime) - double(it->startTime))/(double(it->killTime) - double(it->startTime));
			it->color.val[3] = amount;
			if( i == mDrawPoints.size()-1 )
			{
				std::cout << "amount: " << amount << " " << numer << " " << denom << std::endl;
			}*/
			it ++;
		} else {
			it = mDrawPoints.erase(it);
		}
		i++;
	}

	// Draw line indicating car direction:
	cv::Point2f movement( round( 10*cos( mCarPose.getYaw() ) ), round( 10*sin( mCarPose.getYaw() ) ) );
	cv::Point2f lookAtPos = newPos + movement;
	cv::line( mImageOverlay, newPos, lookAtPos, cv::Scalar( 0, 255, 0, 255 ) );

	mPreviousPos = newPos;
}

void WorldCoordVisualizer::composeImage()
{
	mImage = mImageGrid + mImagePath;
	mImage = mImage + mImageOverlay;
}

tResult WorldCoordVisualizer::sendImage()
{
	cv::cvtColor( mImage, mImageSend, CV_RGBA2RGB );
	cv::flip( mImageSend, mImageFlipped, 0 );

	// Convert back to ADTF image:
	mADTFImage = cvToCImage( mImageFlipped );

	// Ready media type:
	cObjectPtr<IMediaSample> pNewSample;
	RETURN_IF_FAILED(_runtime->CreateInstance(OID_ADTF_MEDIA_SAMPLE, IID_ADTF_MEDIA_SAMPLE, (tVoid**) &pNewSample));
	RETURN_IF_FAILED(pNewSample->AllocBuffer( mOutputFormat.nSize ));
	tVoid* l_pDestBuffer;
	if (IS_OK(pNewSample->WriteLock(&l_pDestBuffer)))
	{
		//if (mOutputFormat.nBitsPerPixel == mInputFormat.nBitsPerPixel )
		//{
			/*cImage::StretchBlit((tUInt8*) iImage.GetBitmap(),
					format,
					(tUInt8*) l_pDestBuffer,
					format,
					0);*/
		//}
		l_pDestBuffer = memcpy( (tUInt8*)l_pDestBuffer, mADTFImage.GetBitmap(), mOutputFormat.nSize );
		pNewSample->Unlock( l_pDestBuffer );
	}
	pNewSample->SetTime(mLastTime);
	RETURN_IF_FAILED( mPinVideoOutput.Transmit(pNewSample) );

	RETURN_NOERROR;
}

