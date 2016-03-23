#ifndef SteeringController_H
#define SteeringController_H

#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>

#include <iostream>

#define OID_ADTF_KACADU_SteeringController "adtf.aadc.kacadu.SteeringController"

class SteeringController : public adtf::cFilter
{
   	ADTF_FILTER(OID_ADTF_KACADU_SteeringController, "KACADU Steering Controller", adtf::OBJCAT_DataFilter);

public:
	//! Constructor
	SteeringController(const tChar* __info);
	//! Destructor
	virtual ~SteeringController();

protected:
    adtf::cInputPin mPinRadiusInput;

    adtf::cInputPin mPinYawInput;

    adtf::cInputPin mPinSpeedInput;

    adtf::cOutputPin mPinServoAngleOutput;

    cObjectPtr<IMediaTypeDescription> mSpeedDescription;
    cObjectPtr<IMediaTypeDescription> mAngleDescription;
    cObjectPtr<IMediaTypeDescription> mRadiusDescription;

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


	//! Send the current calculated position on through the output pin
	tResult sendPosition();

	tFloat32 mYaw;
	bool mFirstYawReceived;

	tFloat32 mX;
	tFloat32 mY;
	tFloat32 mZ;
    tUInt32 mArduinoTimestamp;

	// IDs ("positions") of values in media sample.
	// Must be initialized once after graph setup.
	tBufferID mSignalIDx;
	tBufferID mSignalIDy;
	tBufferID mSignalIDz;
	tBufferID mSignalIDtimestamp;
	tBufferID mSignalServo;
	bool mIDsSignalOutputSet;

	tTimeStamp mLastTime;
	tTimeStamp mLastSendTime;
	tTimeStamp mTimeBetweenUpdates;
};

//*************************************************************************************************
#endif
