
#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>
using namespace adtf;

#include "MonsterFilter.h"

#include <oadrive_control/LateralController.h>
#include <oadrive_core/Types.h>
#include <oadrive_core/Trajectory2d.h>
#include <oadrive_core/Interpolator.h>
#include <oadrive_control/DriverModule.h>
#include <oadrive_obstacle/ProcessUS.h>
//#include <oadrive_missioncontrol/StateMachine.h>
#include <oadrive_missioncontrol/MissionControl.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>

using namespace oadrive::control;
using namespace oadrive::lanedetection;
using namespace oadrive::interface;
using namespace oadrive::missioncontrol;

/// Create filter shell
//ADTF_FILTER_PLUGIN("KACADU Lane Following", OID_ADTF_KACADU_LANEFOLLOWING, LaneFollowing);
ADTF_PLUGIN_BEGIN( KACADU_MonsterFilterPlugin, "KACADU Monster Filter", 0x00)
	// CHANGE: Use OID given in your header, and your class name:
    ADTF_MAP_FILTER(OID_ADTF_KACADU_MONSTERFILTER, MonsterFilter)
ADTF_PLUGIN_END( KACADU_MonsterFilterPlugin )

MonsterFilter::MonsterFilter(const tChar* __info)
	: InheritThroughput(__info)
	, mCarPose( 0, 0, 0 )
	, mLastTime(0)
	, mIDsServoSignalSet(false)
	, mIDsSpeedSignalSet(false)
	, mIDsDrawPositionSet(false)
	, mInterface(NULL)
    , mFrameImageCounter( 0 )
    , mFrameDepthCounter (5)
{
	mDebugOutputMapOnPin = false;
	SetPropertyStr( "Calibration Folder",
			"/home/aadc/AADC/src/aadcUser/config/" );
	SetPropertyStr( "Car Name", "Goffin" );
	SetPropertyBool( "Output Map on Image Pin", false );
	/*SetPropertyFloat( "SPEED_SUPER_SLOW", 0.1 );
	SetPropertyFloat( "SPEED_SLOW", 0.2 );
	SetPropertyFloat( "SPEED_PARKING", 0.3 );
	SetPropertyFloat( "SPEED_FAST", 0.6 );
	SetPropertyFloat( "SPEED_MAX", 1.0 );*/
}

MonsterFilter::~MonsterFilter()
{
	if( mInterface )
		delete mInterface;
}

tResult MonsterFilter::Init(tInitStage eStage, __exception)
{
	// never miss calling the parent implementation!!
	RETURN_IF_FAILED(InheritThroughput::Init(eStage, __exception_ptr))

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

        //input
        //get description for sensor data pins
        tChar const * strInUltraDesc = pDescManager->GetMediaDescription("tUltrasonicStruct");
        RETURN_IF_POINTER_NULL(strInUltraDesc);
        //get mediatype for ultrasonic sensor data pins
        cObjectPtr<IMediaType> pInUltraMedia= new cMediaType(0, 0, 0, "tUltrasonicStruct", strInUltraDesc, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        //get mediatype description
        RETURN_IF_FAILED(pInUltraMedia->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&mUltraInDescription));
        //create pins for ultrasonic sensor data
        RETURN_IF_FAILED(mUltraInput.Create("InUltraStruct", pInUltraMedia, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&mUltraInput));


		// output:
		tChar const * strOutPositionDescription = pDescManager->GetMediaDescription("tSignalValue");
		RETURN_IF_POINTER_NULL(strOutPositionDescription);

		//get mediatype for output data pin:
		cObjectPtr<IMediaType> pOutAngleMedia = new cMediaType(0, 0, 0, "tSignalValue", strOutPositionDescription, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pOutAngleMedia->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&mServoOutDescription));
		RETURN_IF_FAILED(mPinServoAngleOutput.Create("steering_angle", pOutAngleMedia, static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&mPinServoAngleOutput));

		//get mediatype for output data pin:
		cObjectPtr<IMediaType> pOutSpeedMedia = new cMediaType(0, 0, 0, "tSignalValue", strOutPositionDescription, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pOutSpeedMedia->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&mSpeedOutDescription));
		RETURN_IF_FAILED(mPinSpeedOutput.Create("speed", pOutSpeedMedia, static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&mPinSpeedOutput));

		//get mediatype for output position data pin:
		tChar const * strOutDrawPositionDescription = pDescManager->GetMediaDescription("tDrawPositionFloat");
		cObjectPtr<IMediaType> pOutDrawPositionMedia = new cMediaType(0, 0, 0, "tDrawPositionFloat", strOutDrawPositionDescription, IMediaDescription::MDF_DDL_DEFAULT_VERSION); 
		RETURN_IF_FAILED(pOutDrawPositionMedia->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&mDrawPositionDescription));

		RETURN_IF_FAILED(mPinDrawPositionOutput.Create("debug_trajectory_point", pOutDrawPositionMedia, static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&mPinDrawPositionOutput));

		// input jury struct
        tChar const * strDesc1 = pDescManager->GetMediaDescription("tJuryStruct");
        RETURN_IF_POINTER_NULL(strDesc1);
        cObjectPtr<IMediaType> pType1 = new cMediaType(0, 0, 0, "tJuryStruct", strDesc1, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(mPinJuryStructInput.Create("Jury_Struct", pType1, this));
        RETURN_IF_FAILED(RegisterPin(&mPinJuryStructInput));
        RETURN_IF_FAILED(pType1->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&mJuryStructDescription));

        // input maneuver list
        tChar const * strDesc3 = pDescManager->GetMediaDescription("tManeuverList");
        RETURN_IF_POINTER_NULL(strDesc3);
        cObjectPtr<IMediaType> pType3 = new cMediaType(0, 0, 0, "tManeuverList", strDesc3, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(mPinManeuverListInput.Create("Maneuver_List", pType3, this));
        RETURN_IF_FAILED(RegisterPin(&mPinManeuverListInput));
        RETURN_IF_FAILED(pType3->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&mManeuverListDescription));

        // output driver struct
        tChar const * strDesc2 = pDescManager->GetMediaDescription("tDriverStruct");
        RETURN_IF_POINTER_NULL(strDesc2);
        cObjectPtr<IMediaType> pType2 = new cMediaType(0, 0, 0, "tDriverStruct", strDesc2, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(mPinDriverStructOutput.Create("Driver_Struct", pType2, this));
        RETURN_IF_FAILED(RegisterPin(&mPinDriverStructOutput));
        RETURN_IF_FAILED(pType2->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&mDriverStructDescription));

        //get description for bool light signals
        tChar const * strDescBoolSignalValue = pDescManager->GetMediaDescription("tBoolSignalValue");	
        RETURN_IF_POINTER_NULL(strDescBoolSignalValue);	
        //get mediatype for ultrasonic sensor data pins
        cObjectPtr<IMediaType> pTypeBoolSignalValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBoolSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);


        //get mediatype description for output data type
        RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&mDescriptionBool));

        //create pin for headlight bool data
        RETURN_IF_FAILED(mPinOutputHeadLight.Create("headLightEnabled", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&mPinOutputHeadLight));

        //create pin for reverse bool data
        RETURN_IF_FAILED(mPinOutputReverseLight.Create("reverseLightEnabled", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&mPinOutputReverseLight));

        //create pin for brake output data
        RETURN_IF_FAILED(mPinOutputBrakeLight.Create("brakeLightEnabled", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&mPinOutputBrakeLight));

        //create pin for turn right output data
        RETURN_IF_FAILED(mPinOutputTurnRight.Create("turnSignalRightEnabled", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&mPinOutputTurnRight));

        //create pin for turn left output data
        RETURN_IF_FAILED(mPinOutputTurnLeft.Create("turnSignalLeftEnabled", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&mPinOutputTurnLeft));

        //create pin for headlight bool data
        RETURN_IF_FAILED(mPinOutputHazardLight.Create("hazardLightEnabled", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&mPinOutputHazardLight));
		
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

		/*float s;
		oadrive::missioncontrol::SpeedStruct speed;
		s = static_cast<tFloat32>(GetPropertyFloat( "SPEED_SUPER_SLOW" ));
		speed.SPEED_SUPER_SLOW = s;
		s = static_cast<tFloat32>(GetPropertyFloat( "SPEED_SLOW" ));
		speed.SPEED_SLOW = s;
		s = static_cast<tFloat32>(GetPropertyFloat( "SPEED_PARKING" ));
		speed.SPEED_PARKING = s;
		s = static_cast<tFloat32>(GetPropertyFloat( "SPEED_FAST" ));
		speed.SPEED_FAST = s;
		s = static_cast<tFloat32>(GetPropertyFloat( "SPEED_MAX" ));
		speed.SPEED_MAX = s;*/


		tChar mTmpPath[512];
		GetPropertyStr("Calibration Folder",
				mTmpPath, 512 );
		tChar mTmpName[512];
		GetPropertyStr("Car Name",
				mTmpName, 512 );
		mInterface = new Interface( mTmpPath, mTmpName, this );

		mDebugOutputMapOnPin = GetPropertyBool("Output Map on Image Pin");

		mInterface->startDebugDumping( "/tmp/recordedData" );

		// Path for interface debug output: (TODO: Remove!)
		cFileSystem::CreatePath( "/tmp/recordedData/", true );

		mIDsJuryStructSet = false;
		mIDsDriverStructSet = false;

	}
	RETURN_NOERROR;
}

tResult MonsterFilter::Shutdown(tInitStage eStage, __exception)
{
	return InheritThroughput::Shutdown(eStage, __exception_ptr);
}

tResult MonsterFilter::OnPinEvent(IPin* pSource,
		tInt nEventCode,
		tInt nParam1,
		tInt nParam2,
		IMediaSample* pMediaSample)
{
	InheritThroughput::OnPinEvent( pSource, nEventCode, nParam1, nParam2, pMediaSample );

	// first check what kind of event it is
	if( nEventCode == IPinEventSink::PE_MediaSampleReceived )
	{
		if (pSource == &mPinPositionInput)
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

			mCarPose.setX( x );
			mCarPose.setY( y );
			mCarPose.setYaw( yaw );

			mArduinoTimestamp = timeStamp;
			mLastTime = pMediaSample->GetTime();
			mInterface->setCarPose( mCarPose );

			tFloat32 steeringAngle = mInterface->getSteering();
			sendSteeringAngle( steeringAngle + 90 );	// AADC Filters define angle '90' as straight ahead
			tFloat32 speed = mInterface->getSpeed();

			//std::cout << "Speed: " << speed << " Steering: " << steeringAngle << std::endl;
			
			// Safety:
			speed = std::min( speed, 3.0f );
			sendSpeed( speed );		// in m/s
		}
        //process ultrasonic input
        else if(pSource == &mUltraInput)
        {
            // read-out the incoming Media Sample
            __adtf_sample_read_lock_mediadescription(mUltraInDescription, pMediaSample, coder);
            oadrive::obstacle::usSensor distance;
            tSignalValue frontLeft;
            tSignalValue frontCenterLeft;
            tSignalValue frontCenter;
            tSignalValue frontCenterRight;
            tSignalValue frontRight;
            tSignalValue sideLeft;
            tSignalValue sideRight;
            tSignalValue rearLeft;
            tSignalValue rearCenter;
            tSignalValue rearRight;

            //get values from media sample
            coder->Get("tFrontLeft", (tVoid*)&(frontLeft));
            coder->Get("tFrontCenterLeft", (tVoid*)&(frontCenterLeft));
            coder->Get("tFrontCenter", (tVoid*)&(frontCenter));
            coder->Get("tFrontCenterRight", (tVoid*)&(frontCenterRight));
            coder->Get("tFrontRight", (tVoid*)&(frontRight));
            coder->Get("tSideLeft", (tVoid*)&(sideLeft));
            coder->Get("tSideRight", (tVoid*)&(sideRight));
            coder->Get("tRearLeft", (tVoid*)&(rearLeft));
            coder->Get("tRearCenter", (tVoid*)&(rearCenter));
            coder->Get("tRearRight", (tVoid*)&(rearRight));
            //we have to do this because above there is also the time in it.
            distance.frontLeft = frontLeft.f32Value;
            distance.frontCenterLeft = frontCenterLeft.f32Value;
            distance.frontCenter = frontCenter.f32Value;
            distance.frontCenterRight = frontCenterRight.f32Value;
            distance.frontRight = frontRight.f32Value;
            distance.sideLeft = sideLeft.f32Value;
            distance.sideRight = sideRight.f32Value;
            distance.rearLeft = rearLeft.f32Value;
            distance.rearCenter = rearCenter.f32Value;
            distance.rearRight = rearRight.f32Value;

            mInterface->setUsSensor(distance);
        }
        //process the request to the jury struct input pin
		else if (pSource == &mPinJuryStructInput)
		{
			tInt8 i8ActionID = -2;
			tInt16 i16entry = -1;

			{   // focus for sample read lock
				__adtf_sample_read_lock_mediadescription(mJuryStructDescription,pMediaSample,pCoder);

				// get the IDs for the items in the media sample 
				if(!mIDsJuryStructSet)
				{
					pCoder->GetID("i8ActionID", mIDJuryStructI8ActionID);
					pCoder->GetID("i16ManeuverEntry", mIDJuryStructI16ManeuverEntry);
					mIDsJuryStructSet = true;
				}
				pCoder->Get(mIDJuryStructI8ActionID, (tVoid*)&i8ActionID);
				pCoder->Get(mIDJuryStructI16ManeuverEntry, (tVoid*)&i16entry);              
			}

			std::cout << "JuryModule event received: action: " << (int)i8ActionID << " maneuver: " << i16entry << std::endl;
		
			if( mInterface )
				mInterface->setJuryCommand( (juryActions)i8ActionID, i16entry );

		}
		else if (pSource == &mPinManeuverListInput)
		{
			{   // focus for sample read lock
				__adtf_sample_read_lock_mediadescription(mManeuverListDescription,pMediaSample,pCoder);

				std::vector<tSize> vecDynamicIDs;

				// retrieve number of elements by providing NULL as first paramter
				tSize szBufferSize = 0;
				if(IS_OK(pCoder->GetDynamicBufferIDs(NULL, szBufferSize)))
				{
					// create a buffer depending on the size element
					tChar* pcBuffer = new tChar[szBufferSize];
					vecDynamicIDs.resize(szBufferSize);
					// get the dynamic ids (we already got the first "static" size element)
					if (IS_OK(pCoder->GetDynamicBufferIDs(&(vecDynamicIDs.front()), szBufferSize)))
					{
						// iterate over all elements
						for (tUInt32 nIdx = 0; nIdx < vecDynamicIDs.size(); ++nIdx)
						{
							// get the value and put it into the buffer
							pCoder->Get(vecDynamicIDs[nIdx], (tVoid*)&pcBuffer[nIdx]);
						}

						// set the resulting char buffer to the string object
						mManeuverList = std::string( (const tChar*) pcBuffer );
						
						//std::cout << "Received ManeuverList" << std::endl << mManeuverList << std::endl;

						if( mInterface )
						{
							mInterface->setManeuverList( mManeuverList );
						}
					}

					// cleanup the buffer
					delete pcBuffer;
				}

			}

			// trigger loading maneuver list and update the ui
			//TriggerLoadManeuverList();
		}

	}
	RETURN_NOERROR;
}

tResult MonsterFilter::sendSteeringAngle( tFloat32 angle )
{
	//get size of media sample signal value
	cObjectPtr<IMediaSerializer> pSerializerSignalOutput;
	mServoOutDescription->GetMediaSampleSerializer(&pSerializerSignalOutput);
	tInt nSize = pSerializerSignalOutput->GetDeserializedSize();
	//create new media sample for position
	{
		//allocate memory with the size given by the descriptor
		cObjectPtr<IMediaSample> pMediaSampleAngle;
		RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleAngle));
		RETURN_IF_FAILED(pMediaSampleAngle->AllocBuffer(nSize));
		RETURN_IF_FAILED(pMediaSampleAngle->SetTime( mLastTime ) );
		{
			// focus for sample write lock
			//write date to the media sample with the coder of the descriptor
			__adtf_sample_write_lock_mediadescription(mServoOutDescription,pMediaSampleAngle,pCoder);
			if(!mIDsServoSignalSet)
			{
				pCoder->GetID("f32Value", mIDServoSignal);
				mIDsServoSignalSet = true;
			}
			pCoder->Set(mIDServoSignal, (tVoid*)&angle);
		}
		//transmit media sample over output pin
		RETURN_IF_FAILED(mPinServoAngleOutput.Transmit(pMediaSampleAngle));
	}
	RETURN_NOERROR;
}

tResult MonsterFilter::sendSpeed( tFloat32 speed )
{
	//get size of media sample signal value
	cObjectPtr<IMediaSerializer> pSerializerSignalOutput;
	mSpeedOutDescription->GetMediaSampleSerializer(&pSerializerSignalOutput);
	tInt nSize = pSerializerSignalOutput->GetDeserializedSize();
	//create new media sample for position
	{
		//allocate memory with the size given by the descriptor
		cObjectPtr<IMediaSample> pMediaSampleSpeed;
		RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleSpeed));
		RETURN_IF_FAILED(pMediaSampleSpeed->AllocBuffer(nSize));
		RETURN_IF_FAILED(pMediaSampleSpeed->SetTime( mLastTime ) );
		{
			// focus for sample write lock
			//write date to the media sample with the coder of the descriptor
			__adtf_sample_write_lock_mediadescription(mSpeedOutDescription,pMediaSampleSpeed,pCoder);
			if(!mIDsSpeedSignalSet)
			{
				pCoder->GetID("f32Value", mIDSpeedSignal);
				mIDsSpeedSignalSet = true;
			}
			pCoder->Set(mIDSpeedSignal, (tVoid*)&speed);
		}
		//transmit media sample over output pin
		RETURN_IF_FAILED(mPinSpeedOutput.Transmit(pMediaSampleSpeed));
	}
	RETURN_NOERROR;
}

int imgCounter = 0;

cv::Mat MonsterFilter::ProcessImage( cv::Mat& image )
{
    if(  (mFrameImageCounter % 1)==0 )
	{
		mInterface->setCameraImage( image, false );
		if( mDebugOutputMapOnPin )
		{
			mDebugMap = mInterface->generateDebugMap( image.size().width, image.size().height, 3 );
		}
	}
	
	if( mDebugOutputMapOnPin )
	{
		if( !mDebugMap.empty() )
		{
			image = mDebugMap.clone();
		}
	}
	
    mFrameImageCounter ++;

	return image;
}

cv::Mat MonsterFilter::ProcessDepthImage( cv::Mat& image )
{
    if( (mFrameDepthCounter %3)==0)
    {
        mInterface->setDepthImage(image);
    }
    mFrameDepthCounter ++;
    return image;
}

tResult MonsterFilter::sendDebugPoint( cv::Point2f pos, cv::Scalar color, float seconds )
{
	//get size of media sample signal value
	cObjectPtr<IMediaSerializer> pSerializerSignalOutput;
	mDrawPositionDescription->GetMediaSampleSerializer(&pSerializerSignalOutput);
	tInt nSize = pSerializerSignalOutput->GetDeserializedSize();
	//create new media sample for position
	{
		//allocate memory with the size given by the descriptor
		cObjectPtr<IMediaSample> pMediaSample;
		RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));
		RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));
		RETURN_IF_FAILED(pMediaSample->SetTime( mLastTime ) );
		{   // focus for sample write lock
			//write date to the media sample with the coder of the descriptor
			__adtf_sample_write_lock_mediadescription(mDrawPositionDescription,pMediaSample,pCoder);

			// get the IDs for the items in the media sample
			if(!mIDsDrawPositionSet)
			{
				pCoder->GetID("fX", mSignalIDx);
				pCoder->GetID("fY", mSignalIDy);
				pCoder->GetID("fSeconds", mSignalIDseconds); //mLastTime + seconds*10e6);
				pCoder->GetID("ui32Red", mSignalIDred);
				pCoder->GetID("ui32Green", mSignalIDgreen);
				pCoder->GetID("ui32Blue", mSignalIDblue);
				mIDsDrawPositionSet = tTrue;
			}

			tUInt8 b = (color.val[0]);
			tUInt8 g = (color.val[1]);
			tUInt8 r = (color.val[2]);

			pCoder->Set(mSignalIDx, (tVoid*)&pos.x);
			pCoder->Set(mSignalIDy, (tVoid*)&pos.y);
			pCoder->Set(mSignalIDseconds, (tVoid*)&seconds);	// Time to display the point
			pCoder->Set(mSignalIDred, (tVoid*)&r);
			pCoder->Set(mSignalIDgreen, (tVoid*)&g);
			pCoder->Set(mSignalIDblue, (tVoid*)&b);
		}
		//transmit media sample over output pin
		RETURN_IF_FAILED(mPinDrawPositionOutput.Transmit(pMediaSample));
	}
	RETURN_NOERROR;
}

tResult MonsterFilter::transmitBoolValue(cOutputPin* oPin, bool value)
{
    cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    mDescriptionBool->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

    tBool bValue = value;
    tUInt32 ui32TimeStamp = 0;

    //write date to the media sample with the coder of the descriptor
    {
        __adtf_sample_write_lock_mediadescription(mDescriptionBool, pMediaSample, pCoderOutput);    

        // set the id if not already done
        if(!mIDsBoolValueOutput)
        {
            pCoderOutput->GetID("bValue", mIDBoolValueOutput);
            pCoderOutput->GetID("ui32ArduinoTimestamp", mIDArduinoTimestampOutput);
            mIDsBoolValueOutput = tTrue;
        }      

        // set value from sample
        pCoderOutput->Set(mIDBoolValueOutput, (tVoid*)&bValue);     
        pCoderOutput->Set(mIDArduinoTimestampOutput, (tVoid*)&(ui32TimeStamp));     
    }

    pMediaSample->SetTime(_clock->GetStreamTime());

    //transmit media sample over output pin
    oPin->Transmit(pMediaSample);

    RETURN_NOERROR;
}

tResult MonsterFilter::sendState(stateCar stateID, tInt16 i16ManeuverEntry)
{            
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    mDriverStructDescription->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();

    tInt8 value = tInt8(stateID);

    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));
    {   // focus for sample write lock
        __adtf_sample_write_lock_mediadescription(mDriverStructDescription,pMediaSample,pCoder);
        // get the IDs for the items in the media sample 
        if(!mIDsDriverStructSet)
        {
            pCoder->GetID("i8StateID", mIDDriverStructI8StateID);
            pCoder->GetID("i16ManeuverEntry", mIDDriverStructI16ManeuverEntry);
            mIDsDriverStructSet = tTrue;
        }  

        pCoder->Set(mIDDriverStructI8StateID, (tVoid*)&value);
        pCoder->Set(mIDDriverStructI16ManeuverEntry, (tVoid*)&i16ManeuverEntry);
    }      

    pMediaSample->SetTime(_clock->GetStreamTime());
	mPinDriverStructOutput.Transmit(pMediaSample);
	switch (stateID)
	{
		case stateCar_READY:
			LOG_INFO(cString::Format("Driver Module: Send state: READY, Maneuver ID %d",i16ManeuverEntry));
			break;
		case stateCar_RUNNING:
			LOG_INFO(cString::Format("Driver Module: Send state: RUNNING, Maneuver ID %d",i16ManeuverEntry));
			break;
		case stateCar_COMPLETE:
			LOG_INFO(cString::Format("Driver Module: Send state: COMPLETE, Maneuver ID %d",i16ManeuverEntry));
			break;
		case stateCar_ERROR:
			LOG_INFO(cString::Format("Driver Module: Send state: ERROR, Maneuver ID %d",i16ManeuverEntry));
			break;
		case stateCar_STARTUP:
			break;
	}

	RETURN_NOERROR;
}

void MonsterFilter::setJuryState( stateCar state, int manEntry )
{
	// Just send it on:
	sendState( state, manEntry );
}

void MonsterFilter::setLights( enumLight light, bool val )
{
	if( light == HEAD_LIGHT )
	{
		transmitBoolValue( &mPinOutputHeadLight, val );
	} else if( light == REVERSE_LIGHT ) {
		transmitBoolValue( &mPinOutputReverseLight, val );
	} else if( light == BRAKE_LIGHT ) {
		transmitBoolValue( &mPinOutputBrakeLight, val );
	} else if( light == BLINK_RIGHT_LIGHT ) {
		transmitBoolValue( &mPinOutputTurnRight, val );
	} else if( light == BLINK_LEFT_LIGHT ) {
		transmitBoolValue( &mPinOutputTurnLeft, val );
	} else if( light == HAZARD_LIGHT ) {
		transmitBoolValue( &mPinOutputHazardLight, val );
	}
}

void MonsterFilter::reset()
{
  std::cerr << "MosterFilter's reset called, but is not implemented!" << std::endl;
}
