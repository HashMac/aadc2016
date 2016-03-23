#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Convert an ADTF cImage to an OpenCV Mat.
cv::Mat cImageToCV(const cImage& c)
{
	const tInt16 format = c.GetFormat()->nPixelFormat;

	/*std::cout << "Pixel Format: " << c.GetFormat()->nPixelFormat << std::endl;
	std::cout << "Pixel Format 2: " << format << std::endl;
	std::cout << "BitsPerPixel: " << c.GetFormat()->nBitsPerPixel << std::endl;
	std::cout << "BitsPerPixel 2: " << c.GetBitsPerPixel() << std::endl;
	std::cout << "Palette Size: " << c.GetFormat()->nPaletteSize << std::endl;*/
	int type = CV_8UC3;
	if( format == IImage::PF_GREYSCALE_8 ) {
		//std::cout << "CV_8UC1" << std::endl;
		type = CV_8UC1;
	} else if( format == IImage::PF_RGB_888 ) {
		//std::cout << "CV_8UC3" << std::endl;
		type = CV_8UC3;
	} else if( format == IImage::PF_GREYSCALE_16 ) {
		//std::cout << "CV_16UC1" << std::endl;
		type = CV_16UC1;
	} else if( format == IImage::PF_RGBA_8888 ) {
		//std::cout << "CV_8UC4" << std::endl;
		type = CV_8UC4;
	} else if( format == IImage::PF_GREYSCALE_FLOAT32 ) {
		//std::cout << "CV_32FC1" << std::endl;
		type = CV_32FC1;
	} else {
		std::stringstream ss;
		ss << "Wrong PixelFormat in input image: " << format << " (Unsupported by cImageToCV).";
		LOG_ERROR( ss.str().c_str() );
	}

	//std::cout << "Converting adtf pixel format " << format << " to OpenCV " << type << std::endl;

	return cv::Mat(c.GetHeight(),c.GetWidth(),type,c.GetBitmap());
}

// Convert OpenCV image to ADTF image.
// IMPORTANT: Currently only works if depth of image is 8 and it has 3 channels (RGB) or 1 channel (Grayscale).
cImage cvToCImage( const cv::Mat& im )
{
	std::stringstream ss;
	cImage resultIm;
	tInt resultFormat = IImage::PF_UNKNOWN;
	if( im.type() == CV_8UC1 )
		resultFormat = IImage::PF_GREYSCALE_8;
	else if( im.type() == CV_8UC3 )
		resultFormat = IImage::PF_RGB_888;
	else if( im.type() == CV_16UC1 )
		resultFormat = IImage::PF_GREYSCALE_16 ;
	else if( im.type() == CV_8UC4 )
		resultFormat = IImage::PF_RGBA_8888;
	else if( im.type() == CV_32FC1 )
		resultFormat = IImage::PF_GREYSCALE_FLOAT32;		
	else {
		std::stringstream ss;
		ss << "Wrong PixelFormat in input image: " << im.type() << " (Unsupported by cvToCImage).";
		LOG_ERROR( ss.str().c_str() );
		return resultIm;
	}

		/*std::stringstream ss2;
		ss2 << "Creating: " << resultFormat << " " << im.elemSize();
		LOG_WARNING( ss2.str().c_str() );*/

	//std::cout << "Converting OpenCV pixel format " << im.type() << " to ADTF " << resultFormat << std::endl;
	//std::cout << "\tOpenCV elemSize: " << im.elemSize() << std::endl;

	//Create (tInt nWidth, tInt nHeight, tInt nBitsPerPixel, tInt nBytesPerLine=0, const tUInt8 *pBitmap=NULL, tInt nPixelFormat=0)
	tResult res = resultIm.Create(im.size().width, im.size().height, im.elemSize()*8, im.step, NULL, resultFormat);
	if( !res == ERR_NOERROR )
	{
		LOG_ERROR( "Could not create ADTF image in cvToCImage!" );
		return resultIm;
	}

	if( (size_t)resultIm.GetSize() != (size_t)im.step*im.rows )
	{
		LOG_ERROR( "Something went wrong. Sizes of OpenCV image and ADTF image don't match. Cannot copy!" );
		return resultIm;
	}

	/*ss << "Pixel format: " << pixelFormat << " w: " << im.size().width << " h: " << im.size().height;
	ss << " result size: " << resultIm.GetSize() << " source size: " << im.step*im.rows;
	LOG_WARNING( ss.str().c_str() );*/

	// Get a pointer to the raw image data
	tUInt8* rawPtr = resultIm.GetBitmap();
	if( rawPtr == NULL )
	{
		LOG_ERROR( "Bitmap returned nullpointer when converting to ADTF format." );
	} else {
		// Raw data copy: (YUCK!)
		// TODO: Test resultIm.SetBits function
		memcpy(rawPtr, (char*)im.data, im.step * im.rows);
	}

	return resultIm;
}
