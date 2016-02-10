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
	int m_exposure;
	int m_brightness;
	int m_contrast;
	int m_saturation;

	void firstPass(void){
		for (int i = 0; i < (RES_X*RES_Y); i++){
			//do something with pixels here!

			//variables R, G, B, and alpha should be indexed with 4*i to get pixels from original image
			//proc_pixel should be indexed by i to set pixels in processed image
			//the following line would set all the pixels in the
			//processed image to only the green pixels from the original image
			//proc_pixel[i] = G[4*i];

		}
	}
	
	//define user variables here

public:
	USBVision(void){
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
		R = (char*)(raw_info.imageStart);
		proc_pixel = (char*)(proc_info.imageStart);
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

		//initialize remaining variables
		m_exposure = 150;
		m_contrast = 5;
		m_brightness = 200;
		m_saturation = 80;

	}

	USBVision(Int64 exposure, Int64 brightness, Int64 contrast, Int64 saturation){
		USBVision();
		setExposure(exposure);
		setBrightness(brightness);
		setContrast(contrast);
		setSaturation(saturation);
	}

	void printCameraAttribtutes(void){
		printf("number of attributes: %d\n", (int)numOfAttributes);
		IMAQdxEnumerateAttributes2(session, cameraAttributes, &numOfAttributes, "", IMAQdxAttributeVisibilityAdvanced);
		for (unsigned int i = 0; i < numOfAttributes; i++)
			printf("(type %d, writable %d): %s\n", i, cameraAttributes[i].Type, cameraAttributes[i].Writable, cameraAttributes[i].Name);
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
			//you can modify this section for your vision processing
			firstPass();
			CameraServer::GetInstance()->SetImage(processed);

		}

		// stop image acquisition
		IMAQdxStopAcquisition(session);
	}


};

#endif /* SRC_USBVISION_H_ */
