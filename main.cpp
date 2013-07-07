#include <stdio.h>

#include <string>
#include <iostream>
#include "CaliHelper.h"

#include <math.h>

using namespace std;

//-----------------------------SET PARMS------------------------------------------------------////


#define NUMBER_OF_CALIBRATION_IMAGES 10
#define CALIBRATION_BOARD_WIDTH 10
#define CALIBRATION_BOARD_HEIGHT 10
#define DISTANCE_TO_CENTER_MM 10

#define FRINGE_IMAGE_LOCATION1 "CapturePatterns//0001.bmp"
#define FRINGE_IMAGE_LOCATION2 "CapturePatterns//0002.bmp"
#define FRINGE_IMAGE_LOCATION3 "CapturePatterns//0003.bmp"

#define CAPTURE_OUTPUT_DIRECTORY "CaptureImages"

#define CALIBRATION_DIRECTORY "CalibrationImages"
//--------------------------------------------------------------------------------------------////


bool IsCalibration()
{
	string calibration;
	cout<<"Is this Calibration?(y/n)\n";
	cin>>calibration;

	if(strcmp(calibration.c_str(),"y")==0)
	{
		return true;
	}
	else
	{
		return false;
	}
}

int main()
{
	bool cali = IsCalibration();

	if(cali)
	{
		cout<<"Calibration Mode.\n";
		// Check if calibration images exist
		
		unique_ptr<Mat[]> calibrationImages = CaliHelper::GetCalibrationImages(NUMBER_OF_CALIBRATION_IMAGES);
	
		cvNamedWindow("bal");
		for(int i=0;i<NUMBER_OF_CALIBRATION_IMAGES*2;i++)
		{
			int left;
			char outputName[200];

			if(i%2 ==0)
			{
				left = 0;
			}
			else
			{
				left = 1;
			}
			int index = i/2;

			sprintf(outputName,"%s\\%d_%04d.png",CALIBRATION_DIRECTORY,left,index);
			imwrite(outputName,calibrationImages[i]);
		  
			imshow("bal",calibrationImages[i]);
			cvWaitKey(0);
		}
	}
	else
	{
		 	string images[3] = {FRINGE_IMAGE_LOCATION1,FRINGE_IMAGE_LOCATION2,FRINGE_IMAGE_LOCATION3};
			int numberOfImages = 3;
			unique_ptr<Mat[]> CameraImages = CaliHelper::GrabFringeImages(images,numberOfImages);

			for(int i=0;i<numberOfImages*2;i++)
			{
				int left;
				char outputName[200];
				 
				if(i%2 ==0)
				{
					left = 0;
				}
				else
				{
					left = 1;
				}
				int index = i/2;

				CameraImages[i] = CaliHelper::RotateImage(CameraImages[i],180);

				sprintf(outputName,"%s\\%d_%04d.png",CAPTURE_OUTPUT_DIRECTORY,left,index);
				imwrite(outputName,CameraImages[i]);
		  
				imshow("bal",CameraImages[i]);
				cvWaitKey(0);
			}
	}
	return 0;
}




