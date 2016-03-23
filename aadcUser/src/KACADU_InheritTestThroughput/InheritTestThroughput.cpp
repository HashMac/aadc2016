
// CHANGE: your header name:
#include "InheritTestThroughput.h"

ADTF_PLUGIN_BEGIN(MyPluginClass, "My Plugin description", 0x00)
	// CHANGE: Use OID given in your header, and your class name:
    ADTF_MAP_FILTER(OID_ADTF_KACADU_INHERITTESTTHROUGHPUT, InheritTestThroughput)
ADTF_PLUGIN_END(MyPluginClass)

// Change: InheritTestThroughput
InheritTestThroughput::InheritTestThroughput( const tChar* __info )
	: InheritThroughput( __info )
{
}

InheritTestThroughput::~InheritTestThroughput()
{
}

cv::Mat InheritTestThroughput::ProcessImage( cv::Mat& image )
{
	// ... Write your image manipulation in cv here.
    return image;
}
