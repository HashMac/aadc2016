/**
 *
 * ADTF Template Project
 *
 * @file
 * Copyright &copy; Audi Electronics Venture GmbH. All rights reserved
 *
 * $Author: belkera $
 * $Date: 2011-06-30 16:51:21 +0200 (Do, 30 Jun 2011) $
 * $Revision: 26514 $
 *
 * @remarks
 *
 */

// WARNING!!
// ALWAYS INCLUDE OADRIVE BEFORE ADTIF! The Octmap and adtf both define ver evil __test macros!
// They should all be fired for defining these macros.
// Note: If you ignore this warning, many evil things will happen.
#include <oadrive_core/ExtendedPose2d.h>
#include <oadrive_world/ObjectFusion.h>
#include <oadrive_world/EnvironmentDummy.h>
#include <oadrive_world/EnvOctoMap.h>

#include "USView.h"
#include "stdafx.h"

#include <algorithm>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>


/// Create filter shell
ADTF_FILTER_PLUGIN("KACADU US View", OID_ADTF_KACADU_USView, cUSViewFilter);


cUSViewFilter::cUSViewFilter(const tChar* __info)
  : cFilter(__info)
  , mIDsDrawPositionSet(false)
  , mLastTime(0)
  , mSampleCount(0)
{
    SetPropertyStr("US Calibration File",
                "/home/aadc/AADC/src/aadcUser/config/Goffin/calUsComplete.yml" );
    //this file isn't really needed
    SetPropertyStr("Birdview Calibration File",
                "/home/aadc/AADC/src/aadcUser/config/Goffin/BirdviewCal_lowRes.yml" );

}

cUSViewFilter::~cUSViewFilter()
{

}

tResult cUSViewFilter::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

            if (eStage == StageFirst)
    {

        //get the description manager for this filter
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

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

        tChar const * strInPositionDescription = pDescManager->GetMediaDescription("tPositionFloat");
                RETURN_IF_POINTER_NULL(strInPositionDescription);
        //get mediatype for output position data pin:
        cObjectPtr<IMediaType> pInPositionMedia = new cMediaType(0, 0, 0, "tPositionFloat", strInPositionDescription, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pInPositionMedia->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&mPositionDescription));

        RETURN_IF_FAILED(mPinPositionInput.Create("car_pos", pInPositionMedia, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&mPinPositionInput));
        //===================
        //Output
        //===================
        tChar const * strOutPositionDescription = pDescManager->GetMediaDescription("tSignalValue");
                RETURN_IF_POINTER_NULL(strOutPositionDescription);

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
        GetPropertyStr("US Calibration File",
                       mTmpPath, 512 );
        // FIXME no fix path
        mConvertUs.loadCalPoints(mTmpPath);
        mFusion.setUsPath(mTmpPath);
        mEnvOcto.setUsPath(mTmpPath);
        GetPropertyStr("Birdview Calibration File",
                       mTmpPath, 512 );
        mConvertCar.readConfigFile(mTmpPath);
        mEnviroment.setBirdViewPosConv(mTmpPath);
        mFusion.setEnvironment(&mEnviroment);
        mEnvOcto.setEnvironment(&mEnviroment);
    }

    RETURN_NOERROR;
}

tResult cUSViewFilter::Shutdown(tInitStage eStage, __exception)
{
    // In each stage clean up everything that you initiaized in the corresponding stage during Init.
    // Pins are an exception:
    // - The base class takes care of static pins that are members of this class.
    // - Dynamic pins have to be cleaned up in the ReleasePins method, please see the demo_dynamicpin
    //   example for further reference.
    
    if (eStage == StageGraphReady)
    {
        mEnvOcto.save("/tmp/map6.bt");
        std::cout<<"save Map"<<std::endl<<std::endl<<std::endl;

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

tResult cUSViewFilter::OnPinEvent(IPin* pSource,
                                  tInt nEventCode,
                                  tInt nParam1,
                                  tInt nParam2,
                                  IMediaSample* pMediaSample)
{
    // first check what kind of event it is
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        // so we received a media sample, so this pointer better be valid.
        RETURN_IF_POINTER_NULL(pMediaSample);

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
            mCarPos.setX(x);
            mCarPos.setY(y);
            mCarPos.setOrientation(yaw);
        }
        if (pSource == &mUltraInput)
        {
            tUltrasonicStruct distance;
            mLastTime = pMediaSample->GetTime();

            {// focus for sample read lock

                // read-out the incoming Media Sample
                __adtf_sample_read_lock_mediadescription(mUltraInDescription, pMediaSample, coder);

                //get values from media sample
                coder->Get("tFrontLeft", (tVoid*)&(distance.tFrontLeft));
                coder->Get("tFrontCenterLeft", (tVoid*)&(distance.tFrontCenterLeft));
                coder->Get("tFrontCenter", (tVoid*)&(distance.tFrontCenter));
                coder->Get("tFrontCenterRight", (tVoid*)&(distance.tFrontCenterRight));
                coder->Get("tFrontRight", (tVoid*)&(distance.tFrontRight));
                coder->Get("tSideLeft", (tVoid*)&(distance.tSideLeft));
                coder->Get("tSideRight", (tVoid*)&(distance.tSideRight));
                coder->Get("tRearLeft", (tVoid*)&(distance.tRearLeft));
                coder->Get("tRearCenter", (tVoid*)&(distance.tRearCenter));
                coder->Get("tRearRight", (tVoid*)&(distance.tRearRight));

            }
            mEnviroment.setCar(mCarPos);
            std::cout<<"[US-View] "<<mCarPos<<std::endl;
//            mEnvOcto.processUSMeasurement(2, distance.tFrontCenter.f32Value);
//            mEnvOcto.processUSMeasurement(0,distance.tFrontLeft.f32Value);
//            mEnvOcto.processUSMeasurement(1,distance.tFrontCenterLeft.f32Value);
//            mEnvOcto.processUSMeasurement(2,distance.tFrontCenter.f32Value);
//            mEnvOcto.processUSMeasurement(3,distance.tFrontCenterRight.f32Value);
//            mEnvOcto.processUSMeasurement(4,distance.tFrontRight.f32Value);
            //mEnvOcto.processUSMeasurement(9,distance.tSideLeft.f32Value);
            mEnvOcto.processUSMeasurement(5,distance.tSideRight.f32Value);
//            mEnvOcto.processUSMeasurement(8,distance.tRearLeft.f32Value);
//            mEnvOcto.processUSMeasurement(7,distance.tRearCenter.f32Value);
 //           mEnvOcto.processUSMeasurement(6,distance.tRearRight.f32Value);
            mSampleCount++;
            if((mSampleCount%10)==0)
            {
                std::stringstream path;
                path<<"/tmp/octo/"<<mSampleCount<<".png";
                mEnvOcto.saveOccupancyImage(path.str());
		std::stringstream path2;
		path2<<"/tmp/octo/o"<<mSampleCount<<".png";
		mEnvOcto.saveObstacleImage(path2.str(),0.8,0.25);
                //std::cout<<"Path"<<path<<std::endl;
            }

//            mFusion.PredictAll();
//            mFusion.ProcessUSMeasurement(0,distance.tFrontLeft.f32Value);
//            mFusion.ProcessUSMeasurement(1,distance.tFrontCenterLeft.f32Value);
//            mFusion.ProcessUSMeasurement(2,distance.tFrontCenter.f32Value);
//            mFusion.ProcessUSMeasurement(3,distance.tFrontCenterRight.f32Value);
//            mFusion.ProcessUSMeasurement(4,distance.tFrontRight.f32Value);
//            //mFusion.ProcessUSMeasurement(9,distance.tSideLeft.f32Value);
//            mFusion.ProcessUSMeasurement(5,distance.tSideRight.f32Value);
//            mFusion.ProcessUSMeasurement(8,distance.tRearLeft.f32Value);
//            mFusion.ProcessUSMeasurement(7,distance.tRearCenter.f32Value);
//            mFusion.ProcessUSMeasurement(6,distance.tRearRight.f32Value);
//            mEnviroment.printObjects();

//            std::list<EnvObject> *objects;
//            objects = mEnviroment.getObjects();
//            for (std::list<EnvObject>::iterator it = objects->begin(); it != objects->end(); it++){
//                oadrive::core::ExtendedPose2d pos = it->getPose();
//                sendDebugPoint( cv::Point2f(pos.getX(),pos.getY()), CV_RGB( 128,128,128 ), 4 );
//            }

//            oadrive::core::ExtendedPose2d carFrontLeft = mConvertCar.car2World(mCarPos,mConvertUs.transformToCar(0,distance.tFrontLeft.f32Value));
//            oadrive::core::ExtendedPose2d carFrontCenterLeft = mConvertCar.car2World(mCarPos,mConvertUs.transformToCar(1,distance.tFrontCenterLeft.f32Value));
//            oadrive::core::ExtendedPose2d carFrontCenter = mConvertCar.car2World(mCarPos,mConvertUs.transformToCar(2,distance.tFrontCenter.f32Value));
//            oadrive::core::ExtendedPose2d carFrontCenterRight = mConvertCar.car2World(mCarPos,mConvertUs.transformToCar(3,distance.tFrontCenterRight.f32Value));
//            oadrive::core::ExtendedPose2d carFrontRight = mConvertCar.car2World(mCarPos,mConvertUs.transformToCar(4,distance.tFrontRight.f32Value));
//            oadrive::core::ExtendedPose2d carSideLeft = mConvertCar.car2World(mCarPos,mConvertUs.transformToCar(9,distance.tSideLeft.f32Value));
////            std::cout<<distance.tSideLeft.f32Value<<std::endl;
//            oadrive::core::ExtendedPose2d carSideRight = mConvertCar.car2World(mCarPos,mConvertUs.transformToCar(5,distance.tSideRight.f32Value));
//            oadrive::core::ExtendedPose2d carRearLeft = mConvertCar.car2World(mCarPos,mConvertUs.transformToCar(8,distance.tRearLeft.f32Value));
//            oadrive::core::ExtendedPose2d carRearCenter = mConvertCar.car2World(mCarPos,mConvertUs.transformToCar(7,distance.tRearCenter.f32Value));
//            oadrive::core::ExtendedPose2d carRearRight = mConvertCar.car2World(mCarPos,mConvertUs.transformToCar(6,distance.tRearRight.f32Value));
////            std::cout<< "[USView] SensorCenter"<<carFrontCenter<<std::endl;
//            float time = 4.0; // time how long the points are showed
//            float maxDistance = 0.70;
//            if(distance.tFrontLeft.f32Value<maxDistance)
//            sendDebugPoint( cv::Point2f(carFrontLeft.getX(),carFrontLeft.getY()), CV_RGB( 128,64,0 ), time );
//            if(distance.tFrontCenterLeft.f32Value<maxDistance)
//            sendDebugPoint( cv::Point2f(carFrontCenterLeft.getX(),carFrontCenterLeft.getY()), CV_RGB( 128,128,64 ), time );
//            if(distance.tFrontCenter.f32Value<maxDistance)
//            sendDebugPoint( cv::Point2f(carFrontCenter.getX(),carFrontCenter.getY()), CV_RGB( 64,128,0 ), time );
//            if(distance.tFrontCenterRight.f32Value<maxDistance)
//            sendDebugPoint( cv::Point2f(carFrontCenterRight.getX(),carFrontCenterRight.getY()), CV_RGB( 64,128,64 ), time);
//            if(distance.tFrontRight.f32Value<maxDistance)
//            sendDebugPoint( cv::Point2f(carFrontRight.getX(),carFrontRight.getY()), CV_RGB( 128,64,128 ), time );
// //           if(distance.tSideLeft.f32Value<maxDistance)
// //           sendDebugPoint( cv::Point2f(carSideLeft.getX(),carSideLeft.getY()), CV_RGB( 64,64,128 ), time );
//            if(distance.tSideRight.f32Value<maxDistance)
//            sendDebugPoint( cv::Point2f(carSideRight.getX(),carSideRight.getY()), CV_RGB( 64,128,128 ), time );
//            if(distance.tRearLeft.f32Value<maxDistance)
//            sendDebugPoint( cv::Point2f(carRearLeft.getX(),carRearLeft.getY()), CV_RGB( 128,128,128 ), time );
//            if(distance.tRearCenter.f32Value<maxDistance)
//            sendDebugPoint( cv::Point2f(carRearCenter.getX(),carRearCenter.getY()), CV_RGB( 64,64,0 ), time );
//            if(distance.tRearRight.f32Value<maxDistance)
//            sendDebugPoint( cv::Point2f(carRearRight.getX(),carRearRight.getY()), CV_RGB( 128,0,0 ), time );
        }
    }

    RETURN_NOERROR;
}


tResult cUSViewFilter::sendDebugPoint( cv::Point2f pos, cv::Scalar color, float seconds )
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


