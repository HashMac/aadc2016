#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "opencv2/imgcodecs.hpp"
#include <string>
#include <iostream>
#include "BirdViewPosConv.h"

int main( int argc, char** argv )
{
    double carGridX;
    double carGridY;
    int imagex;
    int imagey;
    BirdViewPosConv myKoordinaten("/home/aadc/AADC/src/aadcUser/config/Goffin/BirdviewCal_lowRes.yml");
    myKoordinaten.getCarGrid(372,411,&carGridX,&carGridY);
    //myKoordinaten.getImageGrid(0.3,0.9,&imagex,&imagey);
    std::cout <<"x:"<<carGridX<<"y:"<<carGridY<<std::endl;

return 0;
}


