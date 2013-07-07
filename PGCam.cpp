#include "PGCam.h"


int PGCam::Init(int width, int height, int offsetX,int offsetY)
{
    FlyCapture2::Error error;
    FlyCapture2::PGRGuid guidL,guidR;
    FlyCapture2::BusManager busMgr;
    
    //Getting the GUID of the cam
    error = busMgr.GetCameraFromIndex(0, &guidL);
    if (error != FlyCapture2::PGRERROR_OK)
    {
        error.PrintErrorTrace();
        return -1;
    }

	error = busMgr.GetCameraFromIndex(1, &guidR);
    if (error != FlyCapture2::PGRERROR_OK)
    {
        error.PrintErrorTrace();
        return -1;
    }
    
    // Connect to a camera
    error = camL.Connect(&guidL);
    if (error != FlyCapture2::PGRERROR_OK)
    {
        error.PrintErrorTrace();
        return -1;
    }

	error = camR.Connect(&guidR);
    if (error != FlyCapture2::PGRERROR_OK)
    {
        error.PrintErrorTrace();
        return -1;
    }

	
    //Starting the capture
    error = camL.StartCapture();
    if (error != FlyCapture2::PGRERROR_OK)
    {
        error.PrintErrorTrace();
        return -1;
    }

	error = camR.StartCapture();
    if (error != FlyCapture2::PGRERROR_OK)
    {
        error.PrintErrorTrace();
        return -1;
    }

	bool set = setFormat7(width,  height,  offsetX,  offsetY);
	if(!set)
	{
		return -1;
	}
    

	return 0;


}

vector<IplImage*> PGCam::grabFrame()
{
	IplImage * frameL;
	IplImage * frameR;

    FlyCapture2::Error error;

	FlyCapture2::Image rawImageL,rawImageR;
	camL.RetrieveBuffer(&rawImageL);
	camR.RetrieveBuffer(&rawImageR);

    // Get the raw image dimensions
    FlyCapture2::PixelFormat pixFormat;
    unsigned int rows, cols, stride;
    rawImageL.GetDimensions( &rows, &cols, &stride, &pixFormat );
	
	frameL = cvCreateImage(cvSize(rawImageL.GetCols(), rawImageL.GetRows()), 8, 1);
	frameR = cvCreateImage(cvSize(rawImageL.GetCols(), rawImageL.GetRows()), 8, 1);
    
	// Create a converted image
    FlyCapture2::Image convertedImageL;
	FlyCapture2::Image convertedImageR;

    // Convert the raw image
    error = rawImageL.Convert( FlyCapture2::PIXEL_FORMAT_MONO8, &convertedImageL );
    if (error != FlyCapture2::PGRERROR_OK)
    {
        error.PrintErrorTrace();
        throw "Fly Cap cannot convert.\n";
    }

	error = rawImageR.Convert( FlyCapture2::PIXEL_FORMAT_MONO8, &convertedImageR );
    if (error != FlyCapture2::PGRERROR_OK)
    {
        error.PrintErrorTrace();
        throw "Fly Cap cannot convert.\n";
    }

    //Copy the image into the IplImage of OpenCV
    memcpy(frameL->imageData, convertedImageL.GetData(), convertedImageL.GetDataSize());
	memcpy(frameR->imageData, convertedImageR.GetData(), convertedImageR.GetDataSize());
	
	vector<IplImage*> images;
	images.push_back(frameL);
	images.push_back(frameR);
	

	return images;

}

bool PGCam::setFormat7(int width, int height, int offsetX, int offsetY)
{
  FlyCapture2::Format7ImageSettings settings;
  settings.mode = FlyCapture2::MODE_0;
  settings.width = width;
  settings.height = height;
  settings.offsetX = offsetX;
  settings.offsetY = offsetY;
	settings.pixelFormat = FlyCapture2::PIXEL_FORMAT_RAW8;

  bool valid;
  FlyCapture2::Format7PacketInfo packetInfo;

	if(!_checkLogError(camL.StopCapture()))
		{ return false; }

	if(!_checkLogError(camR.StopCapture()))
		{ return false; }

  if(!_checkLogError(camL.SetFormat7Configuration(
	&settings, packetInfo.recommendedBytesPerPacket)))
	{ return false; }

    if(!_checkLogError(camR.SetFormat7Configuration(
	&settings, packetInfo.recommendedBytesPerPacket)))
	{ return false; }

	// Dont forget to start the capture again
	if(!_checkLogError(camL.StartCapture()))
		{ return false; }
	if(!_checkLogError(camR.StartCapture()))
		{ return false; }

  return true;
}

bool PGCam::_checkLogError(FlyCapture2::Error error)
{
  if (error != FlyCapture2::PGRERROR_OK)
	{ return false; } // Log our error and return false

  //  No error, return true
  return true;
}


