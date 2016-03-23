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
#include "stdafx.h"
#include "EmergencyBreak.h"
#include <algorithm>
#include <iostream>



/// Create filter shell
ADTF_FILTER_PLUGIN("KACADU Emergency Break Filter", OID_ADTF_KACADU_EMERGENCYBREAK, cEmergencyBreakFilter);


cEmergencyBreakFilter::cEmergencyBreakFilter(const tChar* __info):cFilter(__info)
{
	mBreak = false;
    mMessageCounter = 0;
}

cEmergencyBreakFilter::~cEmergencyBreakFilter()
{

}

tResult cEmergencyBreakFilter::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))
    
    if (eStage == StageFirst)
    {

	//get the description manager for this filter
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

	//input
	//get description for sensor data pins
        tChar const * strInSpeedDesc = pDescManager->GetMediaDescription("tSignalValue");	
        RETURN_IF_POINTER_NULL(strInSpeedDesc);
     	//get mediatype for ultrasonic sensor data pins
        cObjectPtr<IMediaType> pInSpeedMedia= new cMediaType(0, 0, 0, "tSignalValue", strInSpeedDesc, IMediaDescription::MDF_DDL_DEFAULT_VERSION); 
	//get mediatype description	
	RETURN_IF_FAILED(pInSpeedMedia->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&mSpeedInDescription));
        
        //create pins for ultrasonic sensor data
        RETURN_IF_FAILED(mSpeedInput.Create("InSpeed", pInSpeedMedia, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&mSpeedInput));

        //get mediatype
        cObjectPtr<IMediaType> pInSpeedControlMedia= new cMediaType(0, 0, 0, "tSignalValue", strInSpeedDesc, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    //get mediatype description
    RETURN_IF_FAILED(pInSpeedControlMedia->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&mSpeedInControlDescription));

        //create pins
        RETURN_IF_FAILED(mSpeedControlInput.Create("InSpeedControl", pInSpeedControlMedia, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&mSpeedControlInput));
        
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
 
	//output
	//get description
        tChar const * strOutSpeedDesc = pDescManager->GetMediaDescription("tSignalValue"); 
	RETURN_IF_POINTER_NULL(strOutSpeedDesc)    
        //get mediatype
        cObjectPtr<IMediaType> pOutSpeedMedia = new cMediaType(0, 0, 0, "tSignalValue", strOutSpeedDesc ,IMediaDescription::MDF_DDL_DEFAULT_VERSION);	
        //get mediatype description
        RETURN_IF_FAILED(pOutSpeedMedia->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&mSpeedOutDescription)); 

        //create pin for output
        RETURN_IF_FAILED(mSpeedOutput.Create("OutSpeed",pOutSpeedMedia, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&mSpeedOutput));

        //get mediatype
        cObjectPtr<IMediaType> pOutControlSpeedMedia = new cMediaType(0, 0, 0, "tSignalValue", strOutSpeedDesc ,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        //get mediatype description
        RETURN_IF_FAILED(pOutSpeedMedia->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&mSpeedOutControlDescription));

        //create pin for output
        RETURN_IF_FAILED(mSpeedControlOutput.Create("OutControlSpeed",pOutControlSpeedMedia, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&mSpeedControlOutput));
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
    }

    RETURN_NOERROR;
}

tResult cEmergencyBreakFilter::Shutdown(tInitStage eStage, __exception)
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

tResult cEmergencyBreakFilter::OnPinEvent(IPin* pSource,
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

        if (pSource == &mSpeedInput)
        {
            tFloat32 speed = 0;
            tUInt32 timeStamp = 0;

            {// focus for sample read lock

                // read-out the incoming Media Sample
                __adtf_sample_read_lock_mediadescription(mSpeedInDescription, pMediaSample, coder);


                //get values from media sample
                coder->Get("f32Value", (tVoid*)&speed);
                coder->Get("ui32ArduinoTimestamp", (tVoid*) &timeStamp);

            }

            if(mBreak){
                speed = 90.0;
            }
            speed = std::max(speed,45.0f);
            transmitSpeed(mSpeedOutput,speed, timeStamp);

        }
        if (pSource == &mSpeedControlInput)
        {
            tFloat32 speed = 0;
            tUInt32 timeStamp = 0;

            {// focus for sample read lock

                // read-out the incoming Media Sample
                __adtf_sample_read_lock_mediadescription(mSpeedInControlDescription, pMediaSample, coder);


                //get values from media sample
                coder->Get("f32Value", (tVoid*)&speed);
                coder->Get("ui32ArduinoTimestamp", (tVoid*) &timeStamp);

            }

            if(mBreak){
                speed = 0.0;
            }
            transmitSpeed(mSpeedControlOutput,speed, timeStamp);
        }
        if (pSource == &mUltraInput)
        {
            tUltrasonicStruct distance;

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

            if(distance.tFrontLeft.f32Value < 0.08 || distance.tFrontCenter.f32Value < 0.1 || distance.tFrontRight.f32Value < 0.08 || distance.tRearCenter.f32Value < 0.08 ) {
                mBreak = true;
                mMessageCounter++;
                if((mMessageCounter%20)==0||mMessageCounter == 0) //not so much annoying Messages
                {
                    std::cout<<"Emergency Break is on!!!"<<std::endl;
                }
            }
            else{
                mBreak = false;
                if(mMessageCounter>0)
                {
                    mMessageCounter = 0;
                    std::cout<<"EmergencyBreak Release"<<std::endl;
                }
            }
        }
    }

    RETURN_NOERROR;
}


// -------------------------------------------------------------------------------------------------
tResult cEmergencyBreakFilter::transmitSpeed(cOutputPin & pin, tFloat32 value, tUInt32 timeStamp) {
// -------------------------------------------------------------------------------------------------
  cObjectPtr<IMediaSample> pMediaSample;
  RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

  cObjectPtr<IMediaSerializer> serializer;
  mSpeedOutDescription->GetMediaSampleSerializer(&serializer);
  tInt size = serializer->GetDeserializedSize();

  RETURN_IF_FAILED(pMediaSample->AllocBuffer(size));
  
  {
    __adtf_sample_write_lock_mediadescription(mSpeedOutDescription, pMediaSample, coder);
    coder->Set("f32Value", (tVoid*) &value);
    coder->Set("ui32ArduinoTimestamp", (tVoid*) &timeStamp);
  }

  pin.Transmit(pMediaSample);
  RETURN_NOERROR;
}


