#ifndef BIRDVIEWCALIBRATION_H
#define BIRDVIEWCALIBRATION_H

#define OID_ADTF_KACADU_BIRDVIEWCALIBRATION "adtf.aadc.kacadu.birdviewcalibration"

#include <opencv2/core/core.hpp>
#include "stdafx.h"
#include "displaywidget.h"

class BirdViewCalibration : public QObject, public cBaseQtFilter
{
   	ADTF_FILTER(OID_ADTF_KACADU_BIRDVIEWCALIBRATION, "KACADU Bird View Calibration", adtf::OBJCAT_DataFilter);
   	Q_OBJECT

protected:
	//! Input of video. This also determines the video format of the output pin!
   	cVideoPin mPinVideoInput;

	//! Video output. Format depends on what is input on mPinVideoInput
   	 cVideoPin mPinVideoOutput;

	//! Current input format
	tBitmapFormat mInputFormat;
	//! Current output format
	tBitmapFormat mOutputFormat;

public:
	//! Constructor
	BirdViewCalibration(const tChar* __info);
	//! Destructor
	virtual ~BirdViewCalibration();



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

	cv::Mat ProcessImage( cv::Mat image );
private:
//User variables
    //! The displayed widget
    DisplayWidget *m_pWidget;
    //!state of calibration
    int m_calibrationState;
	//!ratio of the Calibration Pattern
	double mRatio;
	//! y offset in pixel
	int mYOffset;
	//! Scaling of the picture
	double mScale;
	//!distance of 2 Points in the Calibration pattern in m center to center
	double mDistance2PointM;
	//!distance front of the car to the middle of the first line in the calibration pattern in m
	double mDistanceCarToCalPattern;
	//!coordinates of the bottom left corner of the birdview relative to the center of the front bumber
	cv::Point2f mCoordinatesBirdView;
	//!m per pixel in the Birdview
	double mMPerPixel;
	//!number of Points of the pattern (width)
	int mPointsWidth;
	//!number of Points of the pattern (height)
	int mPointsHeight;

	//! size of the picture
	cv::Size mImgSize;
	//! Path to camera calibration File
	std::string mCameraCalFile;
	//! Path to the complete calibration FIle
	std::string mCompleteCalFile;
    //! Number of succesfull identfied patterns
    int mNumberOfFounds;
    bool mCalibrationFinished;


	//! Calibration Points in the src Picture
	#define numberOfCalPoints 4
	cv::Point2f mCalPoints[numberOfCalPoints];
	//! Destination Vertices in the Bird View
	cv::Point2f mDestVertices[4];



	//! Matrix to disort image
	cv::Mat mCameraMatrix, mDistCoeffs;
	//!Matrix to transform the Image
	cv::Mat mWrapMatrix;


	//!load camera cal file and remove camera disturbance
	cv::Mat unDisortImg(cv::Mat img);
	//!draw calibration Points
	cv::Mat drawCalPoints(cv::Mat img);
	//!calculate the position of the Birdview in the car coordinates
	void calcPosBirdviewInCarGrid();
	//!read Camera Calibration File from Disk
	void readCameraCalFile ();
	//!read complete Calibration File (Camera and Wrap Matrix)
	//void readCompleteCalFile();
	//!save calibration file
	void writeCalFile (const string& path);

	//!search for circle calibration pattern to define the right points
	//@param img image with number of circles x hight number of circles y
	//@return success = true
	bool autoCalPoints(cv::Mat img);

	//!calculate the warpmatrix
	//**This function calculates the wrapmatrix out of 4 given points of a quadrat.	*/
	void calculateWrapMatrix();

	//!transform the image into Birdview
	cv::Mat transform(cv::Mat image);

	//****************************************
	//GUI

	/*! Creates the widget instance*/
    tHandle CreateView();

    /*! Destroys the widget instance*/
    tResult ReleaseView();


    public slots:
    /*! slot for starting the calibration */
    void OnStartCalibration();

    /*! slot for saving the file
    @param qFilename the filename including path where to save
    */
    void OnSaveAs(QString qFilename);

    //!slot for change yOffset
    void OnYOffset(int);

    //!slot for change scale
    void OnScale(int);

    signals:

    /*! send the state to the gui
    @param state the state with the enum
    */
    void sendState(int state);





};

//work arround for int to string
namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}
#endif
