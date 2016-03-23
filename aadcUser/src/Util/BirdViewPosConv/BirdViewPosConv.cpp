/*
 *  Created on: 21.10.2015
 *      Author: peter
 */

#include "BirdViewPosConv.h"

#include <opencv2/core/core.hpp>
#include <oadrive_core/Trajectory2d.h>
#include <oadrive_core/Interpolator.h>
#include <oadrive_core/Pose.h>
#include <oadrive_core/ExtendedPose2d.h>

using namespace oadrive::core;

BirdViewPosConv::BirdViewPosConv(std::string path)
{
	readConfigFile(path);
}

BirdViewPosConv::~BirdViewPosConv()
{
}

ExtendedPose2d BirdViewPosConv::pixel2Car( cv::Point2f pixelPos )
{
	double xCar = pixelPos.x*mMetersPerPixel + mCoordinatesBirdView.getX();
	double yCar = (mImgSize.height-pixelPos.y)*mMetersPerPixel + mCoordinatesBirdView.getY();
	return ExtendedPose2d( xCar, yCar, 0 );
}

/*void BirdViewPosConv::getCarGrid(int xImage, int yImage, double &xCar, double &yCar)
  {
  xCar = xImage*mMPerPixel+mCoordinatesBirdView.x;
  yCar = ((mImgSize.height-yImage)*mMPerPixel+mCoordinatesBirdView.y)+CARLENGTH;
  }*/

cv::Point2f BirdViewPosConv::car2Pixel( ExtendedPose2d pose )
{
	double xImage = (pose.getX()-mCoordinatesBirdView.getX())/mMetersPerPixel;
	double yImage = (-1)*((pose.getY()-mCoordinatesBirdView.getY())/mMetersPerPixel - mImgSize.height);
	return cv::Point2f( xImage, yImage );
}

/*bool BirdViewPosConv::getImageGrid(double xCar, double yCar, int* xImage, int* yImage)
  {
 *xImage = (xCar-mCoordinatesBirdView.x)/mMPerPixel;
 *yImage = (-1)*(((yCar+CARLENGTH)-mCoordinatesBirdView.y)/mMPerPixel -mImgSize.height);
 return true;
 }*/

//ExtendedPose2d BirdViewPosConv::getWorldCordFromExPose( ExtendedPose2d car, ExtendedPose2d point)
ExtendedPose2d BirdViewPosConv::car2World( ExtendedPose2d car, ExtendedPose2d point )
{
	double ang = car.getYaw() - M_PI*0.5;

	double x = point.getX();
	double y = point.getY();

	// Rotate point by angle of car pose:
	ExtendedPose2d rotated( x*cos( ang ) - y*sin( ang ),
			x*sin( ang ) + y*cos( ang ), -point.getYaw() + ang );

	// Add car position:
	ExtendedPose2d pointWorld( rotated.getX() + car.getX(), rotated.getY() + car.getY(), rotated.getYaw() );
		
	/*double carYaw = car.getYaw() - M_PI*0.5;
	//std::cout<<"angleCar"<<carYaw<<std::endl;
	double angleCarToPoint = std::atan2(point.getY(),point.getX());
	//std::cout<<"angleCarToPoint"<<angleCarToPoint<<std::endl;
	double distanceToCar = std::sqrt(point.getY()*point.getY()+point.getX()*point.getX());
	//std::cout<<"DistanceToCar"<<distanceToCar<<std::endl;
	double yTranslation = distanceToCar*std::sin(angleCarToPoint+carYaw);
	//std::cout<<"yTrans"<<yTranslation;
	double xTranslation = distanceToCar*std::cos(angleCarToPoint+carYaw);
	//std::cout<<"  xTrans"<<xTranslation<<std::endl;
	pointWorld.setX(car.getX()+xTranslation);
	pointWorld.setY((car.getY()+yTranslation));*/
	return pointWorld;
}

ExtendedPose2d BirdViewPosConv::pixel2World( ExtendedPose2d car, cv::Point2f pixelPos )
{
	// First, transform to car coordinates:
	ExtendedPose2d poseInCarCoords = pixel2Car( pixelPos );
	// ... and then convert those to global, world coordinates:
	return car2World( car, poseInCarCoords );
}

void BirdViewPosConv::readConfigFile(std::string path)
{
	cv::FileStorage fs2(path, cv::FileStorage::READ);
	if(fs2.isOpened())
	{
		std::cout<<"[BirdViewPosConv] open camera calibration file"<<std::endl;
		cv::FileNode nodeCoordinatesBirdView;

		nodeCoordinatesBirdView = fs2["CoordinatesBirdView"];
		if(nodeCoordinatesBirdView.empty()){
			std::cout<<"[BirdViewPosConv] can't find node nodewarpMatrix please check the file";
		}
		else
		{
			cv::Point2f tmp;
			fs2["CoordinatesBirdView"]>>tmp;
			mCoordinatesBirdView = ExtendedPose2d( tmp.x, tmp.y, 0 );
			fs2["MPerPixel"]>>mMetersPerPixel;
			fs2["ImgSize"]>>mImgSize;
		}
	}
	else
	{
		std::cout<<"[BirdViewPosConv] can't open complete Calibration File"<<std::endl;
	}
	fs2.release();
}
