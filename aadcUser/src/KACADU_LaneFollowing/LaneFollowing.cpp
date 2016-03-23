
#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>
using namespace adtf;

#include "LaneFollowing.h"

#include <oadrive_control/LateralController.h>
#include <oadrive_core/Types.h>
#include <oadrive_core/Trajectory2d.h>
#include <oadrive_core/Interpolator.h>

#include <iostream>


/// Create filter shell
//ADTF_FILTER_PLUGIN("KACADU Lane Following", OID_ADTF_KACADU_LANEFOLLOWING, LaneFollowing);
ADTF_PLUGIN_BEGIN( KACADU_LaneFollowingPlugin, "KACADU Lane Following", 0x00)
	// CHANGE: Use OID given in your header, and your class name:
    ADTF_MAP_FILTER(OID_ADTF_KACADU_LANEFOLLOWING, LaneFollowing)
ADTF_PLUGIN_END( KACADU_LaneFollowingPlugin )

LaneFollowing::LaneFollowing(const tChar* __info)
	: InheritThroughput(__info)
	/*, mX(0)
	, mY(0)
	, mYaw(0)*/
	, mLastTime(0)
	, mIDsServoSignalSet(false)
	, mIDsSpeedSignalSet(false)
	, mIDsDrawPositionSet(false)
{
	SetPropertyStr("Complete Calibration File",
			"/home/aadc/AADC/src/aadcUser/config/Goffin/BirdviewCal_lowRes.yml" );
}

LaneFollowing::~LaneFollowing()
{
}

tResult LaneFollowing::Init(tInitStage eStage, __exception)
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
		tChar mTmpPath[512];
		GetPropertyStr("Complete Calibration File",
				mTmpPath, 512 );
		mLaneFollower.setConfigFile( mTmpPath );
	}
	RETURN_NOERROR;
}

tResult LaneFollowing::Shutdown(tInitStage eStage, __exception)
{
	return InheritThroughput::Shutdown(eStage, __exception_ptr);
}

tResult LaneFollowing::OnPinEvent(IPin* pSource,
		tInt nEventCode,
		tInt nParam1,
		tInt nParam2,
		IMediaSample* pMediaSample)
{
	InheritThroughput::OnPinEvent( pSource, nEventCode, nParam1, nParam2, pMediaSample );

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

		//mX = x;
		//mY = y;
		//mYaw = yaw;
		
		mArduinoTimestamp = timeStamp;
		mLastTime = pMediaSample->GetTime();
	
		/*float randomAngle = (float)(rand() % 60) + 60;
		sendSteeringAngle( 90.0 );		// in degree*/

		sendSpeed( 0.3 );		// in m/s

		//sendSpeed( 0.5 );		// in m/s*/

		mLaneFollower.setPose( x, y, yaw );
		tFloat32 steeringAngle = mLaneFollower.steer();
		sendSteeringAngle( steeringAngle );
	}
	RETURN_NOERROR;
}

tResult LaneFollowing::sendSteeringAngle( tFloat32 angle )
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

tResult LaneFollowing::sendSpeed( tFloat32 speed )
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

cv::Mat LaneFollowing::ProcessImage( cv::Mat& image )
{
	mLaneFollower.setImage( image );
	
	oadrive::core::Trajectory2d traj = mLaneFollower.getTrajectory();
	for( size_t i = 0; i < traj.size(); i++ )
	{
		//std::cout << traj[i].getX() << " " << traj[i].getY() << std::endl;
		sendDebugPoint( cv::Point2f( traj[i].getX(), traj[i].getY() ), CV_RGB( 255,128,0 ), 1 );
	}
	//sendDebugPoint( cv::Point2f( 1, 2 ), CV_RGB( 255,255,0 ), 0 );

	return mLaneFollower.getResultImage();
}

tResult LaneFollowing::sendDebugPoint( cv::Point2f pos, cv::Scalar color, float seconds )
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
