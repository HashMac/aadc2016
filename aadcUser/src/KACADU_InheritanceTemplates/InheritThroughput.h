#ifndef INHERITTHROUGHPUT_H
#define INHERITTHROUGHPUT_H

#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>
#include <opencv2/core/core.hpp>

#include <iostream>

/*! InheritThroughput
 * Inherit from this filter to get a filter which manipulates an OpenCV image and passes it on as an ADTF image.
 */
class InheritThroughput : public adtf::cFilter
{
public:
	//! Constructor
	InheritThroughput(const tChar* __info);
	//! Destructor
	virtual ~InheritThroughput();

	//! Called just before the calibration runs (after properties have been set - highest runlevel reached)
	virtual void Startup() {}

	//! Called every time a new image arrives
	virtual cv::Mat ProcessImage( cv::Mat &image ) = 0;

	//! Called every time a new image arrives
	virtual cv::Mat ProcessDepthImage( cv::Mat &image ) { return image; }

protected:
	//! Input of video. This also determines the video format of the output pin!
   	adtf::cVideoPin mPinVideoInput;
   	adtf::cVideoPin mPinVideoInputDepth;

	//! Video output. Format depends on what is input on mPinVideoInput
   	adtf::cVideoPin mPinVideoOutput;
   	adtf::cVideoPin mPinVideoOutputDepth;

	//! Current input format
	tBitmapFormat mInputFormat;
	tBitmapFormat mInputFormatDepth;
	//! Current output format
	tBitmapFormat mOutputFormat;
	tBitmapFormat mOutputFormatDepth;

	tResult Start(__exception);
	tResult Stop(__exception);

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

};

//*************************************************************************************************
#endif
