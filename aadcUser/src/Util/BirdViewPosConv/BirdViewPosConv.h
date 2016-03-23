/*
 *  Created on: 21.10.2015
 *      Author: peter
 */

#ifndef BIRDVIEWPOSCONV_H_
#define BIRDVIEWPOSCONV_H_


/* Car coordinate system:
The origin is between the car's rear wheels (center of the rear axis).
X increases towards the right of the car, Y increases towards the front of the car.
          ^ y
          |
          |
          |
        / | \
        | | |
        \ |-/---------> x
*/

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "opencv2/imgcodecs.hpp"
#include "string.h"
#include <iostream>
#include "time.h"
#include <oadrive_core/ExtendedPose2d.h>

// Distance from front bumper to the center of the rear axis
#define CARLENGTH 0.502		// in m

class BirdViewPosConv
{
public:
    BirdViewPosConv(std::string path);
    BirdViewPosConv(){};
    ~BirdViewPosConv();

    /*! \brief get from image coordinates to car coordinates
     * \param xImage x coordinate of the Image 0 is the upper left corner
     * \param yImage y coordinate of the image 0 is the upper left corner
     * \param xCar return value for x coordinate in car grid 0 is the middle of back axis
     * \param yCar return Value for y coordinate in car grid 0 is the middle of back axis
     * \return
     */
	//void getCarGrid(int xImage, int yImage, double &xCar, double &yCar);
	oadrive::core::ExtendedPose2d pixel2Car( cv::Point2f input );

    /*! \brief get from image coordinates to car coordinates
     * \param xCar  x coordinate in car grid 0 is the middle of back axis
     * \param yCar y coordinate in car grid 0 is the middle of back axis
     * \param xImage return value for x coordinate of the Image 0 is the upper left corner
     * \param yImage return value for y coordinate of the image 0 is the upper left corner
     * \return true if coordinates are in the image
     */
    //bool getImageGrid(double xCar, double yCar, int* xImage, int* yImage);
	cv::Point2f car2Pixel( oadrive::core::ExtendedPose2d pose );

    oadrive::core::ExtendedPose2d car2World( oadrive::core::ExtendedPose2d car, oadrive::core::ExtendedPose2d point);
    oadrive::core::ExtendedPose2d pixel2World( oadrive::core::ExtendedPose2d car, cv::Point2f pixelPos );

	//! reads the config file
	void readConfigFile(std::string path);

protected:
private:
    //!coordinates of the bottom left corner of the birdview relative to the center of the front bumper
	oadrive::core::ExtendedPose2d mCoordinatesBirdView;
	//!m per pixel in the Birdview
	double mMetersPerPixel;
	//! size of the picture
	cv::Size mImgSize;
};


#endif
