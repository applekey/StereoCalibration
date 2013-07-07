
#include <stdio.h>
#include <cv.h>
#include <highgui.h>
#include <FlyCapture2.h>
#include <string>


#define SAVE_DIRECTORY_RIGHT "C:\\Users\\linda\\Desktop\\RightCamera"
#define RIGHT_ROT  0
#define SAVE_DIRECTORY_LEFT "C:\\Users\\linda\\Desktop\\LeftCamera"
#define LEFT_ROT 0

#define BOARD_SIZE 13

using namespace std;
using namespace cv;


IplImage *rotate_image(IplImage *image, int _90_degrees_steps_anti_clockwise)
{
    IplImage *rotated;

	if(_90_degrees_steps_anti_clockwise ==0)
		return image;


    if(_90_degrees_steps_anti_clockwise != 2)
        rotated = cvCreateImage(cvSize(image->height, image->width), image->depth, image->nChannels);
    else
        rotated = cvCloneImage(image);

    if(_90_degrees_steps_anti_clockwise != 2)
        cvTranspose(image, rotated);

    if(_90_degrees_steps_anti_clockwise == 3)
        cvFlip(rotated, NULL, 1);
    else if(_90_degrees_steps_anti_clockwise == 1)
        cvFlip(rotated, NULL, 0);
    else if(_90_degrees_steps_anti_clockwise == 2)
        cvFlip(rotated, NULL, -1);

    return rotated;
}



int macin()
{
	IplImage *frame = NULL;
    
    FlyCapture2::Error error;
    FlyCapture2::PGRGuid guid;
    FlyCapture2::BusManager busMgr;
    
    //Getting the GUID of the cam
    error = busMgr.GetCameraFromIndex(0, &guid);
    if (error != FlyCapture2::PGRERROR_OK)
    {
        error.PrintErrorTrace();
        return -1;
    }
    
    FlyCapture2::Camera cam;

    // Connect to a camera
    error = cam.Connect(&guid);
    if (error != FlyCapture2::PGRERROR_OK)
    {
        error.PrintErrorTrace();
        return -1;
    }
    
    //Starting the capture
    error = cam.StartCapture();
    if (error != FlyCapture2::PGRERROR_OK)
    {
        error.PrintErrorTrace();
        return -1;
    }
    
    //Get one raw image to be able to calculate the OpenCV window size
    FlyCapture2::Image rawImage;
    cam.RetrieveBuffer(&rawImage);
    
    /* create a window for the video */
    cvNamedWindow( "Original", CV_WINDOW_AUTOSIZE );
    
    //Setting the window size in OpenCV
    frame = cvCreateImage(cvSize(rawImage.GetCols(), rawImage.GetRows()), 8, 1);


	char key = 'c';
	int imageNumber = 1;

    while( key != 'q' ) 
    {   
		key = cvWaitKey( 0 );
        // Start capturing images
        cam.RetrieveBuffer(&rawImage);

        // Get the raw image dimensions
        FlyCapture2::PixelFormat pixFormat;
        unsigned int rows, cols, stride;
        rawImage.GetDimensions( &rows, &cols, &stride, &pixFormat );

        // Create a converted image
        FlyCapture2::Image convertedImage;

        // Convert the raw image
        error = rawImage.Convert( FlyCapture2::PIXEL_FORMAT_MONO8, &convertedImage );
        if (error != FlyCapture2::PGRERROR_OK)
        {
            error.PrintErrorTrace();
            return -1;
        }

        //Copy the image into the IplImage of OpenCV
        memcpy(frame->imageData, convertedImage.GetData(), convertedImage.GetDataSize());

        /* always check */
        if( !frame ) break;




        //Display the original image
        cvShowImage( "Original", frame );

		char imageName[50];
		sprintf(imageName, "\\Image%d.png",imageNumber);
		string directory = SAVE_DIRECTORY_RIGHT + string(imageName);
		imwrite( directory, Mat(frame) );

        // exit if user press 'q' 
		imageNumber++;
        
    }
 
    /* free memory */
    cvDestroyWindow( "Original" );
    cvReleaseImage(&frame);
    // Stop capturing images
    error = cam.StopCapture();
    if (error != FlyCapture2::PGRERROR_OK)
    {
        error.PrintErrorTrace();
        return -1;
    }      
    
    // Disconnect the camera
    error = cam.Disconnect();
    if (error != FlyCapture2::PGRERROR_OK)
    {
        error.PrintErrorTrace();
        return -1;
    }


	//////////////calibrate

	//Left

	//#define NUMBER_REFERENCE_IMAGES 15
	//Mat LeftCaliImages[NUMBER_REFERENCE_IMAGES];

	//for(int i =  0;i<NUMBER_REFERENCE_IMAGES;i++)
	//{
	//	char imageName[80];
	//	sprintf(imageName, "\\Image%d.png",i+1);
	//	string imagePath = SAVE_DIRECTORY_LEFT + string(imageName);
	//	LeftCaliImages[i] = imread(imagePath,CV_LOAD_IMAGE_COLOR);
	//}

	//vector<vector<Point2f> > imagePoints;
 //   Mat cameraMatrix, distCoeffs;
 //   Size imageSize;

	//Size boardSize(BOARD_SIZE,BOARD_SIZE);
	//
	//for(int i =  0;i<NUMBER_REFERENCE_IMAGES;i++)
	//{
	//	bool found = false;
	//	vector<Point2f> pointBuf;
	//	found = findCirclesGrid( LeftCaliImages[i], boardSize, pointBuf, CALIB_CB_ASYMMETRIC_GRID );

	//

	//}





	//Right






    return 0;


}

