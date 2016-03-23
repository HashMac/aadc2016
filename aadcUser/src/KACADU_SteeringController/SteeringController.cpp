
#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>
using namespace adtf;

#include <math.h>

#include "SteeringController.h"

/// Create filter shell
ADTF_FILTER_PLUGIN("KACADU Steering Controller", OID_ADTF_KACADU_SteeringController, SteeringController);

SteeringController::SteeringController(const tChar* __info):cFilter(__info)
	, mYaw(0)
	, mX(0)
	, mY(0)
	, mZ(0)
	, mArduinoTimestamp(0)
	, mIDsSignalOutputSet(false)
	, mLastTime(0)
	, mLastSendTime(0)
	, mTimeBetweenUpdates(0.25e6)
{
}

SteeringController::~SteeringController()
{
}

tResult SteeringController::Init(tInitStage eStage, __exception)
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
			cObjectPtr<IMediaType> pInSpeedMedia = new cMediaType(0, 0, 0, "tSignalValue", strInSpeedDesc, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pInSpeedMedia->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&mSpeedDescription));

			// create and register the input pins:
			RETURN_IF_FAILED(mPinRadiusInput.Create("radius", pInSpeedMedia, static_cast<IPinEventSink*>(this)));
			RETURN_IF_FAILED(RegisterPin(&mPinRadiusInput));
			RETURN_IF_FAILED(mPinYawInput.Create("yaw", pInSpeedMedia, static_cast<IPinEventSink*>(this)));
			RETURN_IF_FAILED(RegisterPin(&mPinYawInput));
			RETURN_IF_FAILED(mPinSpeedInput.Create("CarSpeed", pInSpeedMedia, static_cast<IPinEventSink*>(this)));
			RETURN_IF_FAILED(RegisterPin(&mPinSpeedInput));

			// output:
			tChar const * strOutPositionDescription = pDescManager->GetMediaDescription("tSignalValue");
			RETURN_IF_POINTER_NULL(strOutPositionDescription);
			//get mediatype for output position data pin:
			cObjectPtr<IMediaType> pOutAngleMedia = new cMediaType(0, 0, 0, "tSignalValue", strOutPositionDescription, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pOutAngleMedia->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&mAngleDescription));

			RETURN_IF_FAILED(mPinServoAngleOutput.Create("steering_angle", pOutAngleMedia, static_cast<IPinEventSink*>(this)));
			RETURN_IF_FAILED(RegisterPin(&mPinServoAngleOutput));

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
			mFirstYawReceived = false;
		}

	RETURN_NOERROR;
}

tResult SteeringController::Shutdown(tInitStage eStage, __exception)
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

tResult SteeringController::OnPinEvent(IPin* pSource,
		tInt nEventCode,
		tInt nParam1,
		tInt nParam2,
		IMediaSample* pMediaSample)
{
	// first check what kind of event it is
	if (pSource == &mPinRadiusInput && nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{
		// so we received a media sample, so this pointer better be valid.
		RETURN_IF_POINTER_NULL(pMediaSample);

		tFloat32 radius = 0;

		{// focus for sample read lock

			 // read-out the incoming Media Sample
			__adtf_sample_read_lock_mediadescription(mSpeedDescription, pMediaSample, coder);

			//get values from media sample
			coder->Get("f32Value", (tVoid*)&radius);
		}

		tTimeStamp newTime = pMediaSample->GetTime();

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
			__adtf_sample_read_lock_mediadescription(mSpeedDescription, pMediaSample, coder);

			//get values from media sample
			coder->Get("f32Value", (tVoid*)&yaw);
			coder->Get("ui32ArduinoTimestamp", (tVoid*) &timeStamp);
		}

		//tTimeStamp newTime = pMediaSample->GetTime();
		//std::cout << "time: " << newTime << " yaw: " << yaw << std::endl;

		mYaw = yaw;
		std::cout<<"[SteeringController] Yaw:"<<yaw<<std::endl;
		sendPosition();
	}

    if (pSource == &mPinSpeedInput && nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{
		// so we received a media sample, so this pointer better be valid.
		RETURN_IF_POINTER_NULL(pMediaSample);

		tFloat32 speed = 0;
		tUInt32 timeStamp = 0;

		{// focus for sample read lock

			 // read-out the incoming Media Sample
			__adtf_sample_read_lock_mediadescription(mSpeedDescription, pMediaSample, coder);

			//get values from media sample
			coder->Get("f32Value", (tVoid*)&speed);
			coder->Get("ui32ArduinoTimestamp", (tVoid*) &timeStamp);
			std::cout<<"[SteeringController] Speed:"<<speed<<std::endl;
		}

		mArduinoTimestamp = timeStamp;
		tTimeStamp newTime = pMediaSample->GetTime();

	}
	RETURN_NOERROR;
}

tResult SteeringController::sendPosition()
{
	//get size of media sample signal value
    cObjectPtr<IMediaSerializer> pSerializerSignalOutput;
    mAngleDescription->GetMediaSampleSerializer(&pSerializerSignalOutput);
    tInt nSize = pSerializerSignalOutput->GetDeserializedSize();
    //create new media sample for position
    {
        //allocate memory with the size given by the descriptor
        cObjectPtr<IMediaSample> pMediaSampleAngle;
        RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleAngle));
        RETURN_IF_FAILED(pMediaSampleAngle->AllocBuffer(nSize));
        RETURN_IF_FAILED(pMediaSampleAngle->SetTime( mLastTime ) );
        {   // focus for sample write lock
            //write date to the media sample with the coder of the descriptor
            __adtf_sample_write_lock_mediadescription(mAngleDescription,pMediaSampleAngle,pCoder);
            if(!mIDsSignalOutputSet)
            {
             pCoder->GetID("f32Value", mSignalServo);
            }
            tFloat32 f32Value = 100;

             pCoder->Set(mSignalServo, (tVoid*)&f32Value);


        }
        //transmit media sample over output pin
        RETURN_IF_FAILED(mPinServoAngleOutput.Transmit(pMediaSampleAngle));
    }
	RETURN_NOERROR;
}
