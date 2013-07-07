#include <memory>
#include <iostream>

#include "Ti\LightCrafter.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "WCalibration.h"
#include "PGCam.h"


#define CAMERA_HEIGHT 600
#define CAMERA_WIDTH 800

#define CAMERA_OFFSET_X 200
#define CAMERA_OFFSET_Y 200




class CaliHelper
{

public:
	static unique_ptr<Mat[]> GetCalibrationImages(int numImag);
	static void SaveImages(Mat* images, int frames,string directory);
	static void WriteParmsToFile(CameraParams cameraL,CameraParams cameraR);
	static unique_ptr<Mat[]> GrabFringeImages(string* images,int nFrames);
	static bool RectifyImages(Mat* CameraImages, int numberOfImages, string outputLocation);
	static Mat RotateImage(const Mat& source, double angle);
};
