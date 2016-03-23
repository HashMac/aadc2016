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
#ifndef _TEMPLATE_PROJECT_FILTER_H_
#define _TEMPLATE_PROJECT_FILTER_H_

#define OID_ADTF_KACADU_EMERGENCYBREAK "adtf.util.emergencybreak_filter"


//*************************************************************************************************
class cEmergencyBreakFilter : public adtf::cFilter
{
    ADTF_FILTER(OID_ADTF_KACADU_EMERGENCYBREAK, "KACADU Emergency Break", adtf::OBJCAT_DataFilter);

protected:
    cInputPin    mUltraInput;
    cInputPin    mSpeedInput;
    cInputPin    mSpeedControlInput;
    cOutputPin    mSpeedOutput;
    cOutputPin    mSpeedControlOutput;

    bool mBreak;

    /*! descriptor for ultrasonic sensor data */        
    cObjectPtr<IMediaTypeDescription> mUltraInDescription; 
    /*! descriptor for speed in */
    cObjectPtr<IMediaTypeDescription> mSpeedInDescription; 
    /*! descriptor for speed Out */
    cObjectPtr<IMediaTypeDescription> mSpeedOutDescription; 
    /*! descriptor for speed control in */
    cObjectPtr<IMediaTypeDescription> mSpeedInControlDescription;
    /*! descriptor for speed control Out */
    cObjectPtr<IMediaTypeDescription> mSpeedOutControlDescription;





public:
    cEmergencyBreakFilter(const tChar* __info);
    virtual ~cEmergencyBreakFilter();

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
private:
    int mMessageCounter;
};

//*************************************************************************************************
#endif // _TEMPLATE_PROJECT_FILTER_H_
