/*
 * USBVision.h
 *
 *  Created on: Jan 27, 2016
 *      Author: coreybastarache
 */

#ifndef SRC_USBVISION_H_
#define SRC_USBVISION_H_

#include "WPILib.h"

#define RES_X 640
#define RES_Y 480
#define CAM_NAME "cam0"

class USBVision {

private:
	//imaq types
	IMAQdxSession session;
	Image *frame;
	Image *processed;
	ImageInfo raw_info;
	ImageInfo proc_info;
	IMAQdxError imaqError;

	//image pixel pointers
	char *proc_pixel;
	char proc_array[RES_Y][RES_X];
	char raw_array[RES_Y][RES_X*4];
	char *R, *G, *B, *alpha;

	//attribute variables
	uInt32 numOfAttributes;
	IMAQdxAttributeInformation *cameraAttributes;
	IMAQdxEnumItem m_exposure_mode;
	Int64 m_exposure;
	Int64 m_brightness;
	Int64 m_contrast;
	Int64 m_saturation;

	//user variables
	int cog_x, cog_y, num_of_pixels;

#define THRESHOLD 40
#define CIRCLE_SIZE 100

void firstPass(void){
	cog_x = cog_y = num_of_pixels = 0;

	for (int i = 0; i < (RES_X*RES_Y); i++){
		//do something with pixels here!

		//variables R, G, B, and alpha should be indexed with 4*i to get pixels from original image
		//proc_pixel should be indexed by i to set pixels in processed image
		//the following line would set all the pixels in the
		//processed image to only the green pixels from the original image
		//proc_pixel[i] = G[4*i];

		//int hue = atan2( sqrt(3)*(G[i*4]-B[i*4]), (2*R[i*4])-G[i*4]-B[i*4] );

		if (G[i*4] > THRESHOLD){
			proc_pixel[i] = 255;
			cog_y += i % RES_X;
			cog_x += (int)(i / RES_X);
			num_of_pixels++;
			//printf("cog: (%d,%d) # %d\n", cog_x, cog_y);
			//Wait(0.1);
		}
		else
			proc_pixel[i] = 0;
		//proc_pixel[i] = G[i*4];
	}
	cog_x = cog_x / num_of_pixels;
	cog_y = cog_y / num_of_pixels;
	//printf("final cog: (%d,%d) # %d\n", cog_x, cog_y, num_of_pixels);
	if (num_of_pixels > 500)
		imaqDrawShapeOnImage(processed, processed, {cog_x-100,cog_y-100,200,200}, IMAQ_DRAW_INVERT,IMAQ_SHAPE_OVAL,255);


}

public:
	USBVision(Int64 exposure, Int64 brightness, Int64 contrast, Int64 saturation){
		//initialize image data structure (no size)
		frame = imaqCreateImage(IMAQ_IMAGE_RGB, 0);
		processed = imaqCreateImage(ImageType::IMAQ_IMAGE_U8,0);

		//initialize image with a size
		imaqArrayToImage(frame, &raw_array, RES_X, RES_Y);
		imaqArrayToImage(processed, &proc_array, RES_X, RES_Y);

		//get image pointers and info
		imaqGetImageInfo(frame, &raw_info);
		imaqGetImageInfo(processed, &proc_info);

		//set up pixel pointers
		proc_pixel = (char*)(proc_info.imageStart);
		R = (char*)(raw_info.imageStart);
		G = R + 1;
		B = R + 2;
		alpha = R + 3;

		//the camera name (ex "cam0") can be found through the roborio web interface
		imaqError = IMAQdxOpenCamera(CAM_NAME, IMAQdxCameraControlModeController, &session);

		if(imaqError != IMAQdxErrorSuccess)
		{
			DriverStation::ReportError("IMAQdxOpenCamera error: " + std::to_string((long)imaqError) + "\n");
		}

		imaqError = IMAQdxConfigureGrab(session);

		if(imaqError != IMAQdxErrorSuccess)
		{
			DriverStation::ReportError("IMAQdxConfigureGrab error: " + std::to_string((long)imaqError) + "\n");
		}

		//set the exposure mode to manual
		IMAQdxGetAttribute(session, "CameraAttributes::Exposure::Mode", IMAQdxValueTypeEnumItem, &m_exposure_mode);
		m_exposure_mode.Value = (uInt32)1;
		imaqError = IMAQdxSetAttribute(session, "CameraAttributes::Exposure::Mode", IMAQdxValueTypeEnumItem, m_exposure_mode);
		if (imaqError != 0)
			DriverStation::ReportError("Error setting camera exposure mode: " + std::to_string((long)imaqError) + "\n");

		IMAQdxEnumerateAttributes2(session, NULL, &numOfAttributes,"" , IMAQdxAttributeVisibilityAdvanced);
		cameraAttributes = new IMAQdxAttributeInformation[numOfAttributes];

		setExposure(exposure);
		setBrightness(brightness);
		setContrast(contrast);
		setSaturation(saturation);
		//initialize remaining variables
		printf("%d\n",(int)getExposure());
		printf("%d\n",(int)getBrightness());
		printf("%d\n",(int)getContrast());
		printf("%d\n",(int)getSaturation());
		cog_x = cog_y = num_of_pixels = 0;

		printf("usb vision constructed!\n");
	}

	void printCameraAttribtutes(void){
		printf("number of attributes: %d\n", (int)numOfAttributes);
		IMAQdxEnumerateAttributes2(session, cameraAttributes, &numOfAttributes, "", IMAQdxAttributeVisibilityAdvanced);
		for (unsigned int i = 0; i < numOfAttributes; i++)
			printf("(type %d, writable %d): %s\n", (int)cameraAttributes[i].Type, (int)cameraAttributes[i].Writable, cameraAttributes[i].Name);
	}

	IMAQdxError setBrightness(Int64 brightness){
		//from 30 to 250?
		return IMAQdxSetAttribute(session, "CameraAttributes::Brightness::Value", IMAQdxValueTypeI64, brightness);
	}

	IMAQdxError setExposure(Int64 exposure){
		//from 5 to 250?
		return IMAQdxSetAttribute(session, "CameraAttributes::Exposure::Value", IMAQdxValueTypeI64, exposure);
	}

	IMAQdxError setContrast(Int64 contrast){
		//from 1 to 10?
		return IMAQdxSetAttribute(session, "CameraAttributes::Contrast::Value", IMAQdxValueTypeI64, contrast);
	}

	IMAQdxError setSaturation(Int64 saturation){
		return IMAQdxSetAttribute(session, "CameraAttributes::Saturation::Value", IMAQdxValueTypeI64, saturation);
	}

	Int64 getBrightness(void){
		IMAQdxGetAttribute(session, "CameraAttributes::Brightness::Value", IMAQdxValueTypeI64, &m_brightness);
		return m_brightness;
	}

	Int64 getExposure(void){
		IMAQdxGetAttribute(session, "CameraAttributes::Exposure::Value", IMAQdxValueTypeI64, &m_exposure);
		return m_exposure;
	}

	Int64 getContrast(void){
		IMAQdxGetAttribute(session, "CameraAttributes::Contrast::Value", IMAQdxValueTypeI64, &m_contrast);
		return m_contrast;
	}

	Int64 getSaturation(void){
		IMAQdxGetAttribute(session, "CameraAttributes::Saturation::Value", IMAQdxValueTypeI64, &m_saturation);
		return m_saturation;
	}

	void getRawImage(void){
		// acquire images
		IMAQdxStartAcquisition(session);
		imaqError = IMAQdxGrab(session, frame, true, NULL);

		if(imaqError != IMAQdxErrorSuccess)
		{
			DriverStation::ReportError("IMAQdxGrab error: " + std::to_string((long)imaqError) + "\n");
		}
		else
		{
			CameraServer::GetInstance()->SetImage(frame);
		}

		// stop image acquisition
		IMAQdxStopAcquisition(session);
	}

	void getProcessedImage(void)
	{
		// acquire images
		IMAQdxStartAcquisition(session);
		imaqError = IMAQdxGrab(session, frame, true, NULL);

		if(imaqError != IMAQdxErrorSuccess)
		{
			DriverStation::ReportError("IMAQdxGrab error: " + std::to_string((long)imaqError) + "\n");
		}
		else
		{
			//you can edit this section to run your vision processing
			firstPass();
			CameraServer::GetInstance()->SetImage(processed);

		}

		// stop image acquisition
		IMAQdxStopAcquisition(session);
	}


};

#endif /* SRC_USBVISION_H_ */
