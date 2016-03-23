
#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>
using namespace adtf;

#include "InheritThroughput.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Functions for converting images to OpenCV and back:
#include "adtf2OpenCV.h"

InheritThroughput::InheritThroughput(const tChar* __info):cFilter(__info)
{
	// Copy 0 into all the bytes of mInputFormat and mOutputFormat:
	cMemoryBlock::MemSet(&mInputFormat, 0, sizeof(mInputFormat));
	cMemoryBlock::MemSet(&mOutputFormat, 0, sizeof(mOutputFormat));
	cMemoryBlock::MemSet(&mInputFormatDepth, 0, sizeof(mInputFormatDepth));
	cMemoryBlock::MemSet(&mOutputFormatDepth, 0, sizeof(mOutputFormatDepth));
}

InheritThroughput::~InheritThroughput()
{
}

tResult InheritThroughput::Init(tInitStage eStage, __exception)
{
	// never miss calling the parent implementation!!
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

		// in StageFirst you can create and register your static pins.
		if (eStage == StageFirst)
		{
			// create and register the input pin
			RETURN_IF_FAILED(mPinVideoInput.Create("video_input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
			RETURN_IF_FAILED(RegisterPin(&mPinVideoInput));
			RETURN_IF_FAILED(mPinVideoOutput.Create("video_output", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
			RETURN_IF_FAILED(RegisterPin(&mPinVideoOutput));

			RETURN_IF_FAILED(mPinVideoInputDepth.Create("depth_input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
			RETURN_IF_FAILED(RegisterPin(&mPinVideoInputDepth));
			RETURN_IF_FAILED(mPinVideoOutputDepth.Create("depth_output", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
			RETURN_IF_FAILED(RegisterPin(&mPinVideoOutputDepth));

		}
		else if (eStage == StageNormal)
		{
			// In this stage you would do further initialisation and/or create your dynamic pins.
			// Please take a look at the demo_dynamicpin example for further reference.
		}
		else if (eStage == StageGraphReady)
		{
			// All pin connections have been established in this stage so you can query your pins
			// about their media types and additional meta data.
			// Please take a look at the demo_imageproc example for further reference.

			// Get the image format of the input pin (which is determined by the filter before it)
			cObjectPtr<IMediaType> pType;
			cObjectPtr<IMediaType> pTypeDepth;
			RETURN_IF_FAILED(mPinVideoInput.GetMediaType(&pType));
			RETURN_IF_FAILED(mPinVideoInputDepth.GetMediaType(&pTypeDepth));

			cObjectPtr<IMediaTypeVideo> pTypeVideo;
			cObjectPtr<IMediaTypeVideo> pTypeVideoDepth;
			RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));
			RETURN_IF_FAILED(pTypeDepth->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideoDepth));
			//mInputFormat = *(pTypeVideo->GetFormat());
			//mOutputFormat = *(pTypeVideo->GetFormat());

			const tBitmapFormat* pFormat = pTypeVideo->GetFormat();
			const tBitmapFormat* pFormatDepth = pTypeVideoDepth->GetFormat();
			if (pFormat == NULL)
			{
				LOG_ERROR("No Bitmap information found on pin \"video_input\"");
				RETURN_ERROR(ERR_NOT_SUPPORTED);
			}

			mInputFormat = *pFormat;
			mOutputFormat = *pFormat;
			mInputFormatDepth = *pFormatDepth;
			mOutputFormatDepth = *pFormatDepth;

			// Set the format of the output pin to the same format as the input pin!
			mPinVideoOutput.SetFormat(&mOutputFormat, NULL);
			mPinVideoOutputDepth.SetFormat(&mOutputFormatDepth, NULL);

			// Let children know that we're about to start running:
			Startup();
		}

	RETURN_NOERROR;
}

tResult InheritThroughput::Start(__exception)
{
	return cFilter::Start(__exception_ptr);
}

tResult InheritThroughput::Stop(__exception)
{
	return cFilter::Stop(__exception_ptr);
}


tResult InheritThroughput::Shutdown(tInitStage eStage, __exception)
{
	// In each stage clean up everything that you initiaized in the corresponding stage during Init.
	// Pins are an exception:
	// - The base class takes care of static pins that are members of this class.
	// - Dynamic pins have to be cleaned up in the ReleasePins method, please see the demo_dynamicpin
	//   example for further reference.

	if (eStage == StageGraphReady)
	{
	}
	else if (eStage == StageNormal)
	{
	}
	else if (eStage == StageFirst)
	{
	}

	// call the base class implementation
	return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult InheritThroughput::OnPinEvent(IPin* pSource,
		tInt nEventCode,
		tInt nParam1,
		tInt nParam2,
		IMediaSample* pMediaSample)
{
	// first check what kind of event it is
	if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{
		// New image was received:
		if (pSource == &mPinVideoInput )
		{
			// so we received a media sample, so this pointer better be valid.
			RETURN_IF_POINTER_NULL(pMediaSample);

			// Read image:
			const tVoid* l_pSrcBuffer;
			cImage iImage;
			adtf::cScopedSampleReadLock rl1( pMediaSample, &l_pSrcBuffer );
			RETURN_IF_FAILED(iImage.Attach((tUInt8*)l_pSrcBuffer, &mInputFormat, NULL));

			// Convert to CV image:
			cv::Mat image = cImageToCV(iImage);

			cv::Mat transformed = ProcessImage( image );

			// Convert back to ADTF image:
			iImage = cvToCImage( transformed );

			// Ready media type:
			cObjectPtr<IMediaSample> pNewSample;
			RETURN_IF_FAILED(_runtime->CreateInstance(OID_ADTF_MEDIA_SAMPLE, IID_ADTF_MEDIA_SAMPLE, (tVoid**) &pNewSample));
			RETURN_IF_FAILED(pNewSample->AllocBuffer(mOutputFormat.nSize));
			tVoid* l_pDestBuffer;
			if (IS_OK(pNewSample->WriteLock(&l_pDestBuffer)))
			{
				if (mOutputFormat.nBitsPerPixel == mInputFormat.nBitsPerPixel )
				{
					cImage::StretchBlit((tUInt8*) iImage.GetBitmap(),
							&mInputFormat,
							(tUInt8*) l_pDestBuffer,
							&mOutputFormat,
							0);
				}
				pNewSample->Unlock( l_pDestBuffer );
			}


			pNewSample->SetTime(pMediaSample->GetTime());
			mPinVideoOutput.Transmit(pNewSample);
		}
		else if (pSource == &mPinVideoInputDepth)		// New depth image was received
		{
			// so we received a media sample, so this pointer better be valid.
			RETURN_IF_POINTER_NULL(pMediaSample);

			// Read image:
			const tVoid* l_pSrcBuffer;
			cImage iImage;
			adtf::cScopedSampleReadLock rl1( pMediaSample, &l_pSrcBuffer );
			RETURN_IF_FAILED(iImage.Attach((tUInt8*)l_pSrcBuffer, &mInputFormatDepth, NULL));

			// Convert to CV image:
			cv::Mat image = cImageToCV(iImage);

			cv::Mat transformed = ProcessDepthImage( image );

			// Convert back to ADTF image:
			iImage = cvToCImage( transformed );

			// Ready media type:
			cObjectPtr<IMediaSample> pNewSample;
			RETURN_IF_FAILED(_runtime->CreateInstance(OID_ADTF_MEDIA_SAMPLE, IID_ADTF_MEDIA_SAMPLE, (tVoid**) &pNewSample));
			RETURN_IF_FAILED(pNewSample->AllocBuffer(mOutputFormatDepth.nSize));
			tVoid* l_pDestBuffer;
			if (IS_OK(pNewSample->WriteLock(&l_pDestBuffer)))
			{
				if (mOutputFormatDepth.nBitsPerPixel == mInputFormatDepth.nBitsPerPixel )
				{
					cImage::StretchBlit((tUInt8*) iImage.GetBitmap(),
							&mInputFormatDepth,
							(tUInt8*) l_pDestBuffer,
							&mOutputFormatDepth,
							0);
				}
				pNewSample->Unlock( l_pDestBuffer );
			}

			pNewSample->SetTime(pMediaSample->GetTime());
			mPinVideoOutputDepth.Transmit(pNewSample);
		}
	}
	else if (nEventCode == IPinEventSink::PE_MediaTypeChanged)
	{
		if (pSource == &mPinVideoInput )
		{
			cObjectPtr<IMediaType> pType;
			RETURN_IF_FAILED(mPinVideoInput.GetMediaType(&pType));

			cObjectPtr<IMediaTypeVideo> pTypeVideo;
			RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));

			const tBitmapFormat* pFormat = pTypeVideo->GetFormat();
			if (pFormat != NULL)
			{
				cMemoryBlock::MemCopy(&mInputFormat, pFormat, sizeof(tBitmapFormat));
				cMemoryBlock::MemCopy(&mOutputFormat, pFormat, sizeof(tBitmapFormat));
				mPinVideoOutput.SetFormat(&mOutputFormat, NULL);
			}
		}
		else if( pSource == &mPinVideoInputDepth )
		{
			cObjectPtr<IMediaType> pType;
			RETURN_IF_FAILED(mPinVideoInputDepth.GetMediaType(&pType));

			cObjectPtr<IMediaTypeVideo> pTypeVideo;
			RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));

			const tBitmapFormat* pFormat = pTypeVideo->GetFormat();
			if (pFormat != NULL)
			{
				cMemoryBlock::MemCopy(&mInputFormatDepth, pFormat, sizeof(tBitmapFormat));
				cMemoryBlock::MemCopy(&mOutputFormatDepth, pFormat, sizeof(tBitmapFormat));
				mInputFormatDepth.nWidth = 320;
				mInputFormatDepth.nHeight = 240;
				mOutputFormatDepth.nWidth = 320;
				mOutputFormatDepth.nHeight = 240;
				mPinVideoOutputDepth.SetFormat(&mOutputFormatDepth, NULL);
			}
		}
	}
	RETURN_NOERROR;
}

/*cv::Mat InheritThroughput::ProcessImage( cv::Mat &image )
{
	std::cout << "Processing Template:" << std::endl;
    return image;
}*/

