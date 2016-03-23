#ifndef WORLDCOORDVISUALIZER_H
#define WORLDCOORDVISUALIZER_H

#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>

#include <iostream>
#include <opencv2/core/core.hpp>
#include <oadrive_core/ExtendedPose2d.h>

//#include "../KACADU_LaneFollowing/LaneFollower.h"

#define OID_ADTF_KACADU_WORLDCOORDVISUALIZER "adtf.aadc.kacadu.worldcoordvisualizer"

struct DebugPoint
{
	cv::Point2f pos;
	cv::Scalar color;
	tTimeStamp startTime;	// Time when point was received (initial display)
	tTimeStamp killTime;	// Time when point should be removed (startTime + displayTime)
};

class WorldCoordVisualizer : public adtf::cFilter
{
   	ADTF_FILTER(OID_ADTF_KACADU_WORLDCOORDVISUALIZER, "KACADU World Coord Visualizer", adtf::OBJCAT_DataFilter);

public:
	//! Constructor
	WorldCoordVisualizer(const tChar* __info);
	//! Destructor
	virtual ~WorldCoordVisualizer();

protected:
    adtf::cInputPin mPinPositionInput;

    adtf::cInputPin mPinDrawPositionInput;

	adtf::cVideoPin mPinVideoOutput;

	//! Output format
	tBitmapFormat mOutputFormat;
    cObjectPtr<IMediaTypeDescription> mPositionDescription; 
    cObjectPtr<IMediaTypeDescription> mDrawPositionDescription; 

	//! Called multiple times during setup
	tResult Init(tInitStage eStage, __exception);

	//! Called multiple times during shutdown
	tResult Shutdown(tInitStage eStage, __exception);

	    /*! Called for every packet event
		 * \note Also called when media type changed!
		 * \param nEventCode IPinEventSing::PE_MediaTypeChanged or IPinEventSink::PE_MediaSampleReceived
		 * \param pMediaSample The new packet */
   	tResult OnPinEvent(adtf::IPin* pSource,
                       tInt nEventCode,
                       tInt nParam1,
                       tInt nParam2,
                       adtf::IMediaSample* pMediaSample);
private:
	
	void updateImage();

	void composeImage();

	//! Send the current image through the output pin
	tResult sendImage();

	oadrive::core::ExtendedPose2d mCarPose;

    tUInt32 mArduinoTimestamp;

	// IDs ("positions") of values in media sample.
	// Must be initialized once after graph setup.
	/*tBufferID mSignalIDx;
	tBufferID mSignalIDy;
	tBufferID mSignalIDz;
	tBufferID mSignalIDtimestamp;
	bool mIDsSignalOutputSet;*/

	tTimeStamp mLastTime;
	tTimeStamp mLastSendTime;
	tTimeStamp mTimeBetweenUpdates;

	unsigned int mOutputWidth;
	unsigned int mOutputHeight;
	unsigned int mImageCenter;
	float mMetersToPixels;

	//! Result image:
	cv::Mat mImageGrid;
	cv::Mat mImagePath;
	cv::Mat mImageOverlay;
	cv::Mat mImage;
	cv::Mat mImageSend;
	cv::Mat mImageFlipped;

	cv::Point2f mPreviousPos;

	cImage mADTFImage;

	std::vector<DebugPoint> mDrawPoints;

	//LaneFollower mLaneFollower;
};

//*************************************************************************************************
#endif
