

#include <memory>

#include <FlyCapture2.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

class PGCam
{
private:
	FlyCapture2::Camera camL,camR;
	IplImage * frame;

	bool setFormat7( int width, int height, int offsetX, int offsetY);
	bool  _checkLogError(FlyCapture2::Error error);

public:
	int Init(int width, int height, int offsetX,int offsetY);
	vector<IplImage*> grabFrame();

};

