
#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>
using namespace adtf;

#include <cmath>

#include "WorldCoordCalculator.h"

/// Create filter shell
ADTF_FILTER_PLUGIN("KACADU World Coord Calculator", OID_ADTF_KACADU_WORLDCOORDCALCULATOR, WorldCoordCalculator);

WorldCoordCalculator::WorldCoordCalculator(const tChar* __info):cFilter(__info)
	, mYaw(0)
	, mX(0)
	, mY(0)
	, mArduinoTimestamp(0)
	, mIDsSignalOutputSet(false)
	, mLastTime(0)
	//, mLastSendTime(0)
	//, mTimeBetweenUpdates(0.25e6)
	, mAngleInitialized(true)
	, mInitialAngle(0)
	, mMinRadiusInvert(1.25)
	, mLastYaw(0)
{
    SetPropertyBool("Reset Angle at startup", tFalse);
	mTwoPi = 2.0*M_PI;
}

WorldCoordCalculator::~WorldCoordCalculator()
{
}

tResult WorldCoordCalculator::Init(tInitStage eStage, __exception)
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
			tChar const * strInSpeedDesc = pDescManager->GetMediaDescription("tSignalValue");
			RETURN_IF_POINTER_NULL(strInSpeedDesc);
			//get mediatype for input sensor data pins
            cObjectPtr<IMediaType> pInLastDistanceMedia = new cMediaType(0, 0, 0, "tSignalValue", strInSpeedDesc, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
            RETURN_IF_FAILED(pInLastDistanceMedia->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&mLastDistanceDescription));

			// create and register the input pins:
            RETURN_IF_FAILED(mPinLastDistanceInput.Create("last_distance", pInLastDistanceMedia, static_cast<IPinEventSink*>(this)));
            RETURN_IF_FAILED(RegisterPin(&mPinLastDistanceInput));
            RETURN_IF_FAILED(mPinYawInput.Create("yaw", pInLastDistanceMedia, static_cast<IPinEventSink*>(this)));
			RETURN_IF_FAILED(RegisterPin(&mPinYawInput));

			// output:
			tChar const * strOutPositionDescription = pDescManager->GetMediaDescription("tPositionFloat");
			RETURN_IF_POINTER_NULL(strOutPositionDescription);
			//get mediatype for output position data pin:
			cObjectPtr<IMediaType> pOutPositionMedia = new cMediaType(0, 0, 0, "tPositionFloat", strOutPositionDescription, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pOutPositionMedia->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&mPositionDescription));

			RETURN_IF_FAILED(mPinPositionOutput.Create("car_pos", pOutPositionMedia, static_cast<IPinEventSink*>(this)));
			RETURN_IF_FAILED(RegisterPin(&mPinPositionOutput));

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
			mIDsSignalOutputSet = false;

			mInitialAngle = 0;
			bool resetAngle = GetPropertyBool("Reset Angle at startup");
			if( resetAngle )
			{
				mAngleInitialized = false;
			}
			else
			{
				mAngleInitialized = true;
			}
		}

	RETURN_NOERROR;
}

tResult WorldCoordCalculator::Shutdown(tInitStage eStage, __exception)
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

tResult WorldCoordCalculator::OnPinEvent(IPin* pSource,
		tInt nEventCode,
		tInt nParam1,
		tInt nParam2,
		IMediaSample* pMediaSample)
{
	// first check what kind of event it is
    if (pSource == &mPinLastDistanceInput && nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{
		// so we received a media sample, so this pointer better be valid.
		RETURN_IF_POINTER_NULL(pMediaSample);

        tFloat32 lastDistance = 0;
		tUInt32 timeStamp = 0;

		{// focus for sample read lock

			 // read-out the incoming Media Sample
            __adtf_sample_read_lock_mediadescription(mLastDistanceDescription, pMediaSample, coder);

			//get values from media sample
            coder->Get("f32Value", (tVoid*)&lastDistance);
			coder->Get("ui32ArduinoTimestamp", (tVoid*) &timeStamp);
		}

		mArduinoTimestamp = timeStamp;
		tTimeStamp newTime = pMediaSample->GetTime();

        updatePosition( newTime, lastDistance );

	}
	// first check what kind of event it is
	if (pSource == &mPinYawInput && nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{
		// so we received a media sample, so this pointer better be valid.
		RETURN_IF_POINTER_NULL(pMediaSample);

		tFloat32 yaw = 0;
		tUInt32 timeStamp = 0;

		{// focus for sample read lock

			 // read-out the incoming Media Sample
            __adtf_sample_read_lock_mediadescription(mLastDistanceDescription, pMediaSample, coder);

			//get values from media sample
			coder->Get("f32Value", (tVoid*)&yaw);
			coder->Get("ui32ArduinoTimestamp", (tVoid*) &timeStamp);
		}

		// Convert to radians:
        double yawDouble = yaw;
        yawDouble = yawDouble*M_PI/180.0f;

		if( !mAngleInitialized )
		{
            mInitialAngle = yawDouble;		// let the car face upwards in the WorldCoordVisualizer
			mAngleInitialized = true;
			mLastYaw = yawDouble;
		}

        mYaw = yawDouble - mInitialAngle;
		// Warp into range of [0,2*pi):
        mYaw = (mYaw - mTwoPi * floor( mYaw / mTwoPi ));

	}
	RETURN_NOERROR;
}

void WorldCoordCalculator::updatePosition( tTimeStamp currentTime, tFloat32 lastDistance )
{

    double mDiffX = lastDistance*cos( mYaw );
    double mDiffY = lastDistance*sin( mYaw );
    mX += mDiffX;
    mY += mDiffY;
//    std::cout<<"X: "<<mX<<" Y: "<<mY<<std::endl;
   if(abs(mLastYaw-mYaw)>(abs(mDiffX+mDiffY)*mMinRadiusInvert*7+0.02))
   {
	std::cout<<"[WorldCoordCalculator] Warning: Yaw rate is not stable. Diff is: "<<abs(mLastYaw-mYaw)<<" Maximum allowed: "<< abs(mDiffX+mDiffY)*mMinRadiusInvert*7 <<std::endl;
   }
   mLastYaw = mYaw;
    sendPosition();

    mLastTime = currentTime;
}

tResult WorldCoordCalculator::sendPosition()
{
	//get size of media sample signal value
    cObjectPtr<IMediaSerializer> pSerializerSignalOutput;
    mPositionDescription->GetMediaSampleSerializer(&pSerializerSignalOutput);
    tInt nSize = pSerializerSignalOutput->GetDeserializedSize();
    //create new media sample for position
    {
        //allocate memory with the size given by the descriptor
        cObjectPtr<IMediaSample> pMediaSamplePosition;
        RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSamplePosition));
        RETURN_IF_FAILED(pMediaSamplePosition->AllocBuffer(nSize));
        RETURN_IF_FAILED(pMediaSamplePosition->SetTime( mLastTime ) );
        {   // focus for sample write lock
            //write date to the media sample with the coder of the descriptor
            __adtf_sample_write_lock_mediadescription(mPositionDescription,pMediaSamplePosition,pCoder);

            // get the IDs for the items in the media sample
            if(!mIDsSignalOutputSet)
            {

                pCoder->GetID("fX", mSignalIDx);
                pCoder->GetID("fY", mSignalIDy);
                pCoder->GetID("fYaw", mSignalIDyaw);
                pCoder->GetID("ui32ArduinoTimestamp", mSignalIDtimestamp);
                mIDsSignalOutputSet = tTrue;
            }
            tFloat32 Xsend = mX;
            tFloat32 YSend = mY;
            tFloat32 YawSend = mYaw;
            pCoder->Set(mSignalIDx, (tVoid*)&Xsend);
            pCoder->Set(mSignalIDy, (tVoid*)&YSend);
            pCoder->Set(mSignalIDyaw, (tVoid*)&YawSend);
            pCoder->Set(mSignalIDtimestamp, (tVoid*)&mArduinoTimestamp);
        }
        //transmit media sample over output pin
        RETURN_IF_FAILED(mPinPositionOutput.Transmit(pMediaSamplePosition));
    }
	RETURN_NOERROR;
}
