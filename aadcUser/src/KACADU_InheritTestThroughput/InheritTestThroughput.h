// CHANGE: Rename to your filename:
#ifndef INHERITTESTTHROUGHPUT_H
#define INHERITTESTTHROUGHPUT_H

#include "InheritThroughput.h"

#include <opencv2/core/core.hpp>

// CHANGE: Rename "INHERITANCETEST" to your filter name, rename "inheritancetest" to your filter name
#define OID_ADTF_KACADU_INHERITTESTTHROUGHPUT "adtf.aadc.kacadu.inherittestthroughput"

// CHANGE: Change class name:
class InheritTestThroughput : public InheritThroughput
{
	// CHANGE: insert OID from above, change "Inheritance Test"
   	ADTF_FILTER(OID_ADTF_KACADU_INHERITTESTTHROUGHPUT,
		"KACADU Inheritance Throughput Test", adtf::OBJCAT_DataFilter);

	public:
	// CHANGE:
		InheritTestThroughput(const tChar* __info);
		~InheritTestThroughput();

    	cv::Mat ProcessImage( cv::Mat &image );
};

#endif
