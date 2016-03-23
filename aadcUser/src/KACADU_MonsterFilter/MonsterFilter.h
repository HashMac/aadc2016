#ifndef KACADU_LANEFOLLOWING_H
#define KACADU_LANEFOLLOWING_H

// ADTF header
#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>
//#include "arduinoProtocol.h"
#include "aadc_enums.h"
#include "aadc_structs.h"

#include <oadrive_core/ExtendedPose2d.h>
//#include <oadrive_control/DriverModule.h>
//#include <oadrive_control/DriverModule.h>
//#include <oadrive_lanedetection/HaarFilter.h>
//#include <oadrive_lanedetection/StreetPatcher.h>
#include <oadrive_interface/Interface.h>
#include <oadrive_missioncontrol/IControl4MC.h>
//#include <oadrive_trajectory/TrajUtil.h>

#include <opencv2/flann/timer.h>

#include "InheritThroughput.h"
#define OID_ADTF_KACADU_MONSTERFILTER "adtf.aadc.kacadu.monsterfilter"

class MonsterFilter : public InheritThroughput, public oadrive::missioncontrol::IControl4MC
{
    ADTF_FILTER(OID_ADTF_KACADU_MONSTERFILTER, "KACADU Monster Filter", adtf::OBJCAT_DataFilter);

	public:
		MonsterFilter(const tChar* __info);
		~MonsterFilter();

    	cv::Mat ProcessImage( cv::Mat &image );
    	cv::Mat ProcessDepthImage( cv::Mat &image );

		/*! Receives jury state signals from mission control passes them on to other filters.
		 * Implements IControl4MC. */
		void setJuryState( stateCar state, int manEntry );

		/*! Receives light commands and passes them on to other filters.
		 * Implements IControl4MC. */
		void setLights( enumLight light, bool val );

		oadrive::util::Timer* getTimer() {
			std::cerr << "MonsterFilter's getTimer called, but is not implemented!" << std::endl;
			return new oadrive::util::Timer();
		}
		cv::Mat getLastBirdViewImage() {
			std::cerr << "MonsterFilter's getLastBirdViewImage called, but is not implemented!" << std::endl;
			return cv::Mat( cv::Size(1,1), CV_8UC1 );
		}

		void setNewOrigin( oadrive::core::ExtendedPose2d& newOrigin ) {
			std::cerr << "MosterFilter's setNewOrigin called, but is not implemented!" << std::endl;
		}

    void reset();

	protected:

		//! Called multiple times during setup
		tResult Init(tInitStage eStage, __exception);

		//! Called multiple times during shutdown
		tResult Shutdown(tInitStage eStage, __exception);

		/*! Called for every packet event
		 * \note Also called when media type changed!
		 * \param nEventCode IPinEventSing::PE_MediaTypeChanged or IPinEventSink::PE_MediaSampleReceived
		 * \param pMediaSample The new packet */
		tResult OnPinEvent(IPin* pSource,
				tInt nEventCode,
				tInt nParam1,
				tInt nParam2,
				IMediaSample* pMediaSample);

		adtf::cInputPin mPinPositionInput;
		cObjectPtr<IMediaTypeDescription> mPositionDescription; 

        cInputPin    mUltraInput;
        cObjectPtr<IMediaTypeDescription> mUltraInDescription;

		adtf::cOutputPin mPinServoAngleOutput;
		cObjectPtr<IMediaTypeDescription> mServoOutDescription;
		adtf::cOutputPin mPinSpeedOutput;
		cObjectPtr<IMediaTypeDescription> mSpeedOutDescription;

		adtf::cOutputPin mPinDrawPositionOutput;
		cObjectPtr<IMediaTypeDescription> mDrawPositionDescription; 

		/*! input pin for the run command */
		cInputPin mPinJuryStructInput;
		/*! Coder Descriptor for input jury struct */
		cObjectPtr<IMediaTypeDescription> mJuryStructDescription;

		/*! output pin for state from driver*/
		cOutputPin        mPinDriverStructOutput;
		/*! Coder Descriptor */
		cObjectPtr<IMediaTypeDescription> mDriverStructDescription;

		/*! input pin for the maneuver list*/
		cInputPin        mPinManeuverListInput;
		/* Coder description */
		cObjectPtr<IMediaTypeDescription> mManeuverListDescription;

		/*! the output pin to send the value for speed controller */
		cOutputPin     mPinOutputSpeedController;

		/*! the output pin to send the value for steering controller */
		cOutputPin     mPinOutputSteeringController;

		/*! the output pin to toggle the head light*/
		cOutputPin     mPinOutputHeadLight;

		/*! the output pin to toggle the reverse light*/
		cOutputPin     mPinOutputReverseLight;

		/*! the output pin to toggle the brake light*/
		cOutputPin     mPinOutputBrakeLight;

		/*! the output pin to toggle the turn right light*/
		cOutputPin     mPinOutputTurnRight;

		/*! the output pin to toggle the turn left light*/
		cOutputPin     mPinOutputTurnLeft;

		/*! the output pin to toggle the head light*/
		cOutputPin     mPinOutputHazardLight;

		/*! Coder Descriptor */
		cObjectPtr<IMediaTypeDescription> mDescriptionBool;

		/*! the id for the bool value output of the media description */
		tBufferID mIDBoolValueOutput;     

		/*! the id for the arduino timestamp output of the media description */
		tBufferID mIDArduinoTimestampOutput; 

		/*! indicates if bufferIDs were set */
		tBool mIDsBoolValueOutput;

	private:

		/*! Send steering angle to servo.
		 * \param angle Angle in Degree(!)	*/	
		tResult sendSteeringAngle( tFloat32 angle );

		/*! Send speed in m/s */
		tResult sendSpeed( tFloat32 speed );

		tResult sendDebugPoint( cv::Point2f pos, cv::Scalar color, float seconds );

		tResult sendState(stateCar stateID, tInt16 i16ManeuverEntry);

		/*! helper function to send a bool signal via the specified pin
		  @param pin the destination pin to use
		  @param value the new value.
		 */
		tResult transmitBoolValue(cOutputPin* pin, tBool value);

		oadrive::core::ExtendedPose2d mCarPose;

		tUInt32 mArduinoTimestamp;
		tTimeStamp mLastTime;
        tTimeStamp mLastTimeSendToInterface;
		bool mIDsServoSignalSet;
		tBufferID mIDServoSignal;
		bool mIDsSpeedSignalSet;
		tBufferID mIDSpeedSignal;

		bool mIDsDrawPositionSet;
		tBufferID mSignalIDx;
		tBufferID mSignalIDy;
		tBufferID mSignalIDseconds;
		tBufferID mSignalIDred;
		tBufferID mSignalIDgreen;
		tBufferID mSignalIDblue;

		/*! indicates if bufferIDs were set */
		bool mIDsJuryStructSet;
		/*! the id for the i8StateID of the media description */
		tBufferID mIDJuryStructI8ActionID; 
		/*! the id for the i16ManeuverEntry of the media description data */
		tBufferID mIDJuryStructI16ManeuverEntry;

		/*! indicates if bufferIDs were set */
		bool mIDsDriverStructSet;
		/*! the id for the i8StateID of the media description */
		tBufferID mIDDriverStructI8StateID; 
		/*! the id for the i16ManeuverEntry of the media description data */
		tBufferID mIDDriverStructI16ManeuverEntry;

		oadrive::interface::Interface* mInterface;
		//oadrive::util::BirdViewPosConv mCoordConverter;

		std::string mManeuverList;

		cvflann::StartStopTimer mTimer;

        unsigned int mFrameImageCounter;
        unsigned int mFrameDepthCounter;
		//std::vector<oadrive::lanedetection::Patch> mPatches;

		bool mDebugOutputMapOnPin;
		cv::Mat mDebugMap;
};

#endif
