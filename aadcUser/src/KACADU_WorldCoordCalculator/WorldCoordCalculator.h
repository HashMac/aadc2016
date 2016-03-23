#ifndef WORLDCOORDCALCULATOR_H
#define WORLDCOORDCALCULATOR_H

#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>

#include <iostream>

#define OID_ADTF_KACADU_WORLDCOORDCALCULATOR "adtf.aadc.kacadu.worldcoordcalculator"

class WorldCoordCalculator : public adtf::cFilter
{
   	ADTF_FILTER(OID_ADTF_KACADU_WORLDCOORDCALCULATOR, "KACADU World Coord Calculator", adtf::OBJCAT_DataFilter);

public:
	//! Constructor
	WorldCoordCalculator(const tChar* __info);
	//! Destructor
	virtual ~WorldCoordCalculator();

protected:
    adtf::cInputPin mPinLastDistanceInput;

    adtf::cInputPin mPinYawInput;

    adtf::cOutputPin mPinPositionOutput;

    cObjectPtr<IMediaTypeDescription> mLastDistanceDescription;
    cObjectPtr<IMediaTypeDescription> mPositionDescription;

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

	//! Calculate the current position by adding up the old position and new values.	
    void updatePosition(tTimeStamp currentTime, tFloat32 lastDistance );

	//! Send the current calculated position on through the output pin
	tResult sendPosition();

    double mYaw;

    double mX;
    double mY;
    tUInt32 mArduinoTimestamp;

	// IDs ("positions") of values in media sample.
	// Must be initialized once after graph setup.
	tBufferID mSignalIDx;
	tBufferID mSignalIDy;
	tBufferID mSignalIDyaw;
	tBufferID mSignalIDtimestamp;
	bool mIDsSignalOutputSet;

	tTimeStamp mLastTime;

	//! Will be set to true once the first angle has been read in
	bool mAngleInitialized;
	//! If resetting of angle is active, then the first received angle will be stored:
    double mInitialAngle;

	float mTwoPi;
	double mMinRadiusInvert;
        double mLastYaw;

};

//*************************************************************************************************
#endif
