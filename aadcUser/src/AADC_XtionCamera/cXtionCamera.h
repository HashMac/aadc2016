/**********************************************************************
* Copyright (c) 2013 BFFT Gesellschaft fuer Fahrzeugtechnik mbH.
* All rights reserved.
**********************************************************************
* $Author:: spiesra $  $Date:: 2015-05-13 08:29:07#$ $Rev:: 35003   $
**********************************************************************/



#ifndef _XTION_CAPTURE_FILTER_HEADER_
#define _XTION_CAPTURE_FILTER_HEADER_



#define OID_ADTF_XTION_CAPTURE "adtf.aadc.aadc_xtionCamera"

/*!
This filter grabs the video streams from the xtion camera by using the openni library. The RGB video stream is transmitted on the output pin Video_RGB and the depth image stream is transmitted on the pin Depth_Image. 
All the settings are managed with the configuration file set in the property Configuration File.  The file is as the following:
<?xml version="1.0" encoding="iso-8859-1" standalone="yes"?>
<xtionSettings xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
	<depthImage>
		<x_resolution>320</x_resolution>
		<y_resolution>240</y_resolution>
		<fps>30</fps>
	</depthImage>
	<colorImage>
		<x_resolution>640</x_resolution>
		<y_resolution>480</y_resolution>
		<fps>30</fps>
	</colorImage>	
	<options>
		<setDepthColorSync>0</setDepthColorSync>
		<setRegistration>0</setRegistration>		
		<setDepthInMillimeter>0</setDepthInMillimeter>
		<setAutoExposure>0</setAutoExposure>
		<setAutoWhiteBalance>1</setAutoWhiteBalance>
	</options>		
</xtionSettings>

A Sample file xtionSettings.xml is located in the folder configuration_files.

The parameter in „setDepthInMillimeter“ sets the scale of the values in the depth image. If set to “0” the format of pixel is directly passed from OpenNi, if set to “1” the flag in OpenNi is set to “openni::PIXEL_FORMAT_DEPTH_1_MM” so the unit of the pixels is millimeter.

*/
class cXtionCamera : public adtf::cFilter, public adtf::IKernelThreadFunc
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_XTION_CAPTURE, "AADC Xtion Camera", OBJCAT_CameraDevice, "AADC Xtion Camera", 1, 1, 0, "");    
    public:
        cXtionCamera(const tChar*);
        virtual ~cXtionCamera();
        tResult Init(tInitStage eStage, __exception=NULL);
        tResult Start(ucom::IException** __exception_ptr=NULL);
        tResult Stop(ucom::IException** __exception_ptr=NULL);
        tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr=NULL);
    protected:
        /*! the threadfunc which handles the reading and sending of the data
        @param pThread
        @param pvUserData
        @param szUserData
        */
        tResult ThreadFunc(cKernelThread* pThread, tVoid* pvUserData, tSize szUserData);
        
        /*! 
        the function for reading the data
        */
        tResult ReadData();
        
        /*!
        function to load an xml to set the settings of the xtion
        */
        tResult LoadConfiguration();
        
        tInt        m_nWidthVideo;                /**< width of the stream */
        tInt        m_nHeightVideo;                /**< height of the stream */
        tInt        m_nWidthDepth;                /**< width of the depth stream */
        tInt        m_nHeightDepth;                /**< height of the depth stream */
        tInt        m_FPSVideo;                    /**< frames per second of the video stream */
        tInt        m_FPSDepth;                    /**< frames per second of the depth stream */
        
        cVideoPin    m_oRGBVideo;                /**<  outputpin for the RGB video */
        cVideoPin    m_oDepthImage;                /**<  outputpin for the depth images */
        
        tBitmapFormat    m_sBitmapFormat;        /**< bitmap format for the RGB video */
        tBitmapFormat    m_sBitmapFormatDepth;    /**< bitmap format for the depth video */
        
        tBool           m_setRegistration;        /**< enable flag for setRegistration */
        tBool           m_setDepthColorSync;    /**< enable flag for setDepthColorSync */
        tBool           m_setDepthInMillimeter;     /**< if true the values in the depth image are given in millimeter*/
        tBool           m_setAutoExposure;     /**< if true the values in the depth image are given in millimeter*/
        tBool           m_setAutoWhiteBalance;     /**< if true the values in the depth image are given in millimeter*/

        cKernelThread    m_oWorker;                /**< the worker for the thread */
        
        xtionGrabber    m_xtionGrabber;            /**< the instance of the grabber (only linux) */

        tInt        m_GrabberTimeoutMs;            /**< the time out for the grabber */        
        
        cFilename m_fileConfig;                    /**< holds the xml file for the supporting points */
};

#endif // _XTION_CAPTURE_FILTER_HEADER_
