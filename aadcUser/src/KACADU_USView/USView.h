/**
 *
 * ADTF Template Project Filter.
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

#ifndef _US_VIEW_FILTER_H_
#define _US_VIEW_FILTER_H_

#include <oadrive_control/LateralController.h>
#include <oadrive_core/Types.h>
#include <oadrive_core/Trajectory2d.h>
#include <oadrive_core/Interpolator.h>
#include <oadrive_util/ConvertUs.h>
#include <oadrive_util/BirdViewPosConv.h>
#include <oadrive_core/ExtendedPose2d.h>
#include <oadrive_world/ObjectFusion.h>
#include <oadrive_world/EnvironmentDummy.h>
#include <oadrive_core/Pose.h>
#include <oadrive_world/EnvOctoMap.h>
#include "stdafx.h"

#define OID_ADTF_KACADU_USView "adtf.util.USView_filter"
using namespace oadrive::world;

//*************************************************************************************************
class cUSViewFilter : public adtf::cFilter
{
    ADTF_FILTER(OID_ADTF_KACADU_USView, "KACADU US View", adtf::OBJCAT_DataFilter);

public:
    cUSViewFilter(const tChar* __info);
    virtual ~cUSViewFilter();

protected:
    tResult Init(tInitStage eStage, __exception);
    tResult Shutdown(tInitStage eStage, __exception);

    tResult transmitSpeed(cOutputPin & pin, tFloat32 value, tUInt32 timeStamp);

    // implements IPinEventSink
    tResult OnPinEvent(IPin* pSource,
                       tInt nEventCode,
                       tInt nParam1,
                       tInt nParam2,
                       IMediaSample* pMediaSample);

    cInputPin    mUltraInput;
    /*! descriptor for ultrasonic sensor data */
    cObjectPtr<IMediaTypeDescription> mUltraInDescription;

    //Car Postion input
    adtf::cInputPin mPinPositionInput;
    cObjectPtr<IMediaTypeDescription> mPositionDescription;

    //visualisation Output
    adtf::cOutputPin mPinDrawPositionOutput;
    cObjectPtr<IMediaTypeDescription> mDrawPositionDescription;
private:

    tResult sendDebugPoint( cv::Point2f pos, cv::Scalar color, float seconds );
    oadrive::util::ConvertUs mConvertUs;
    oadrive::util::BirdViewPosConv mConvertCar;
    oadrive::core::ExtendedPose2d mCarPos;
    EnvironmentDummy mEnviroment;
    ObjectFusion mFusion;
    EnvOctoMap mEnvOcto;

    bool mIDsDrawPositionSet;
    tBufferID mSignalIDx;
    tBufferID mSignalIDy;
    tBufferID mSignalIDseconds;
    tBufferID mSignalIDred;
    tBufferID mSignalIDgreen;
    tBufferID mSignalIDblue;
    tTimeStamp mLastTime;
    int mSampleCount;
};

//*************************************************************************************************
#endif // _US_VIEW_FILTER_H_
