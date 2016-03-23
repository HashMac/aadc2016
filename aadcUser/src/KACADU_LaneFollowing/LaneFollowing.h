#ifndef KACADU_LANEFOLLOWING_H
#define KACADU_LANEFOLLOWING_H

// ADTF header
#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>

#include <oadrive_lanefollower/LaneFollower.h>

#include "InheritThroughput.h"
#define OID_ADTF_KACADU_LANEFOLLOWING "adtf.aadc.kacadu.lanefollowing"

class LaneFollowing : public InheritThroughput
{
    ADTF_FILTER(OID_ADTF_KACADU_LANEFOLLOWING, "KACADU Lane Following", adtf::OBJCAT_DataFilter);

	public:
		LaneFollowing(const tChar* __info);
		~LaneFollowing();

    	cv::Mat ProcessImage( cv::Mat &image );

	protected:

		//! Called multiple times during setup
		tResult Init(tInitStage eStage, __exception);

		//! Called multiple times during shutdown
		tResult Shutdown(tInitStage eStage, __exception);

		/*! Called for every packet event
		 * \note Also called when media type changed!
		 * \param nEventCode IPinEventSing::PE_MediaTypeChanged or IPinEventSink::PE_MediaSampleReceived
		 * \param pMediaSample The new packet */
		tResult OnPinEvent(IPin* pSource,
				tInt nEventCode,
				tInt nParam1,
				tInt nParam2,
				IMediaSample* pMediaSample);

		adtf::cInputPin mPinPositionInput;
		cObjectPtr<IMediaTypeDescription> mPositionDescription; 

		adtf::cOutputPin mPinServoAngleOutput;
		cObjectPtr<IMediaTypeDescription> mServoOutDescription;
		adtf::cOutputPin mPinSpeedOutput;
		cObjectPtr<IMediaTypeDescription> mSpeedOutDescription;

		adtf::cOutputPin mPinDrawPositionOutput;
		cObjectPtr<IMediaTypeDescription> mDrawPositionDescription; 

	private:

		/*! Send steering angle to servo.
		 * \param angle Angle in Degree(!)	*/	
		tResult sendSteeringAngle( tFloat32 angle );

		/*! Send speed in m/s */
		tResult sendSpeed( tFloat32 speed );

		tResult sendDebugPoint( cv::Point2f pos, cv::Scalar color, float seconds );

		tFloat32 mX;
		tFloat32 mY;
		tFloat32 mYaw;

		tUInt32 mArduinoTimestamp;
		tTimeStamp mLastTime;
		bool mIDsServoSignalSet;
		tBufferID mIDServoSignal;
		bool mIDsSpeedSignalSet;
		tBufferID mIDSpeedSignal;

		LaneFollower mLaneFollower;

		bool mIDsDrawPositionSet;
		tBufferID mSignalIDx;
		tBufferID mSignalIDy;
		tBufferID mSignalIDseconds;
		tBufferID mSignalIDred;
		tBufferID mSignalIDgreen;
		tBufferID mSignalIDblue;
};

#endif
