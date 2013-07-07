#include <memory.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>




enum CALIB_SUCCESS {
	CALIB_FAILURE,
	CALIB_SUCCESS_FRAME,
	CALIB_SUCCESS_TOTAL,
	CALIB_SUCCESS_1_ONLY,
	CALIB_SUCCESS_2_ONLY
};

typedef struct {
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;
	cv::Rect validRoi;
	cv::Mat rMapX;
	cv::Mat rMapY;
	cv::Mat Q;
	cv::Mat R;
	cv::Mat P;
	cv::Mat PRinv;
} CameraParams;


class Calibration {
public:
	Calibration(int width, int height, float mDistCenterToCenterInMM = 10);
	bool Calibration::init(int mWidth, int mHeight, float mDistCenterToCenterInMM = 10);
	CALIB_SUCCESS Calibration::addImage(cv::Mat image1Mat, cv::Mat image2Mat, bool invert = false);
	std::vector<cv::Point3f> Calibration::calcObjectPoints();
	const static int maxFrames = 10;
	CameraParams CameraParams1;
	CameraParams CameraParams2;
	cv::Mat canvas;
	CALIB_SUCCESS processAll(void);

	const int getCurrentFrame1(){return currentFrame1;};
	const int getCurrentFrame2(){return currentFrame2;};
	const int getCurrentFrame (){return currentFrame ;};
private:
	//Left camera
	std::vector<std::vector<cv::Point3f>> objectPoints1;
	std::vector<std::vector<cv::Point2f>> imagePoints1;
	std::vector<std::vector<cv::Point2f>> imagePoints1All;
	int currentFrame1;

	//Right camera
	std::vector<std::vector<cv::Point3f>> objectPoints2;
	std::vector<std::vector<cv::Point2f>> imagePoints2;
	std::vector<std::vector<cv::Point2f>> imagePoints2All;
	int currentFrame2;

	cv::Ptr<cv::FeatureDetector> blobDetector;

	int height;
	int width;
	int currentFrame;
	std::vector<cv::Point3f> objectPointsEach;
	std::vector<std::vector<cv::Point3f>> objectPointsAll;
	cv::Size boardSize;
	float distCenterToCenterInMM;
	std::vector<cv::Mat> rvecs, tvecs; // I don't give a damn about these
	std::vector<cv::Point2f> pointBuffer1, pointBuffer2;
	cv::Mat R, T, E, F, P1, P2, Q;
	cv::Mat image1, image2;
};
