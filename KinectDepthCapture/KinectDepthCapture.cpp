// KinectDepthCapture.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <fstream>
#include <Kinect.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <Windows.h>

using namespace cv;
using namespace std;

template<class Interface>
inline void SafeRelease(Interface *&pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL) {
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}


int main()
{
	//Acquire Sensor
	IKinectSensor *Psensor;
	HRESULT hResult = S_OK;
	hResult = GetDefaultKinectSensor(&Psensor);
	if (FAILED(hResult)) {
		cerr << "Error: GetDefaultKinectSensor" << endl;
		return -1;
	}

	//Open Sensor
	hResult = Psensor->Open();
	if (FAILED(hResult)) {
		cerr << "Error: IKinectSensor :: Open()" << endl;
		return -1;
	}

	//Acquire Source
	IDepthFrameSource * PDepthSource;
	hResult = Psensor->get_DepthFrameSource(&PDepthSource);
	if (FAILED(hResult)) {
		cerr << "Error: IKinectSensor :: Get_DethFrameSource()" << endl;
		return -1;
	}

	//Acquire Reader
	IDepthFrameReader * PDepthReader;
	hResult = PDepthSource->OpenReader(&PDepthReader);
	if (FAILED(hResult)) {
		cerr << "Error: IDepthFrameSource :: OpenReader()" << endl;
		return -1;
	}

	//Kinect Depth Resolution
	int width = 512;
	int height = 424;

	//Starting x,y coordinates
	int xPos = 0;
	int yPos = 120;

	//resolution of region of interest
	int videoWidth = width - xPos;
	int videoHeight = height - yPos - 30;

	int frameCount = 0;
	unsigned int bufferSize = width * height * sizeof(unsigned short);

	namedWindow("Depth");

	//Create image mats
	Mat bufferMat(height, width, CV_16UC1);

	Mat depthMat(height, width, CV_8UC1);
	Mat prevFrame;

	float prevMotion = 0;
	int numSavedImages = 0;

	time_t now = time(0);
	tm ltm;
	localtime_s(&ltm, &now);

	string fileName = to_string(ltm.tm_year + 1900) + "_"
		+ to_string(ltm.tm_mon + 1) + "_"
		+ to_string(ltm.tm_mday);

	//Open VideoWriter
	VideoWriter outputVideo;
	string videoFileName = fileName + "_f0.avi";
	outputVideo.open(videoFileName, CV_FOURCC('D', 'I', 'V', '3'), 30, Size(videoWidth, videoHeight), false);

	if (!outputVideo.isOpened()) {
		cerr << "Could not open the output video for write." << endl;
		return -1;
	}

	ofstream myFile;
	myFile.open(fileName + ".csv");
	myFile << "Motion \n";

	//Process Depth Stream
	while (1) 
	{ 
		IDepthFrame * PDepthFrame = nullptr;
		hResult = PDepthReader->AcquireLatestFrame(&PDepthFrame);

		if (SUCCEEDED(hResult)) {

			hResult = PDepthFrame->AccessUnderlyingBuffer(&bufferSize, reinterpret_cast<UINT16**> (&bufferMat.data));
			if (SUCCEEDED(hResult)) {

				//Isolate Region of Interest
				Mat currentFrame = bufferMat(Rect(xPos, yPos, videoWidth, videoHeight));
				currentFrame.convertTo(depthMat, CV_8U, -1.0, 1300.0f);
				
				float totalMotion = 0;

				//adaptive threshold
				if (frameCount > 0) {
					int nRows = prevFrame.rows;
					int nCols = prevFrame.cols;

					if (prevFrame.isContinuous()) {
						nCols *= nRows;
						nRows = 1;
					}

					int i, j;

					unsigned short* p1;
					unsigned short* p2;
					for (i = 0; i < nRows; ++i) {
						p1 = prevFrame.ptr<unsigned short>(i);
						p2 = currentFrame.ptr<unsigned short>(i);
						for (j = 0; j < nCols; ++j) {
							float diff = abs(p1[j] - p2[j]);
							if (diff < 25) {
								p1[j] = p2[j];
							}
							else {
								totalMotion += diff;
							}
						}
					}
				}
				
				if (abs(totalMotion - prevMotion) > 100000) {
					numSavedImages = 150;
					cout << "Motion Detected: " << totalMotion << endl;
				}


				imshow("Depth", depthMat);

				if (numSavedImages > 1) {
					outputVideo.write(depthMat);
					numSavedImages--;
				}
				else if (numSavedImages == 1) {
					outputVideo.write(depthMat);
					//outputVideo.release();
					numSavedImages--;
					string fileName = "f" + to_string(frameCount) + ".avi";
					outputVideo.open(fileName, CV_FOURCC('D', 'I', 'V', '3'), 30, Size(videoWidth, videoHeight), false);
				}

				myFile << totalMotion << endl;
				
				prevFrame = currentFrame.clone();
				prevMotion = totalMotion;
				frameCount++;

			}
		}
		SafeRelease(PDepthFrame);
		if (waitKey(30) == VK_ESCAPE) {
			break;
		}
	}


	SafeRelease(PDepthSource);
	SafeRelease(PDepthReader);
	if (Psensor) {
		Psensor->Close();
	}
	SafeRelease(Psensor);
	cv::destroyAllWindows();

    return 0;
}

