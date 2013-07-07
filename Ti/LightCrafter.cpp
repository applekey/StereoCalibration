#include "LightCrafter.h"


LightCrafter::LightCrafter() : IsConnected(false)
{
  Commander = unique_ptr<LCR_Commander>(new LCR_Commander());
  Connect();
  StaticDisplayMode();
}

int LightCrafter::GetHeight(void)
{
  //static image has to be a bmp at 608 x 684
  return 684;
}

int LightCrafter::GetWidth(void)
{
  //static image has to be a bmp at 608 x 684
  return 608;
}

bool LightCrafter::Connect()
{
  // Try and connect to the LCR
  if( !Commander->Connect_LCR(LCR_Default_IP,LCR_Default_PORT) )
	{ cout<<"Cannot Connect to LCR.\n"; }
  else
  {
	cout<<"Connected To LCR.\n";
	IsConnected = true;
  }
  
  return IsConnected;
}

bool LightCrafter::Disconnect()
{

  if(!Commander->Disconnect_LCR())
	{ cout<< "Could not disconnect from the LCR.\n"; }
  else
  {
	cout<<"Disconnected from LCR.\n";
	IsConnected = false;
  }

  return IsConnected;
}

bool LightCrafter::StaticDisplayMode()
{
  DisplayMode displayMode = StaticImageMode;
  bool modeChanged = Commander->SetDisplayMode(displayMode);
	
  if(!modeChanged)
	{ cout << "Could not change display mode to Static.\n"; }
  else
	{ cout << "Mode changed to Static Display.\n"; }
	  
  return modeChanged;
}

bool LightCrafter::SequenceDisplayMode()
{
  DisplayMode displayMode = PatternSequenceDisplay;
  bool modeChanged = Commander->SetDisplayMode(displayMode);
	
  if(!modeChanged)
	{ cout << "Could not change display mode to Static.\n"; }
  else
	{ cout << "Mode changed to Static Display.\n"; }
	  
  return modeChanged;
}



bool LightCrafter::ProjectImage(cv::Mat image)
{
  cvWaitKey(0);

  CvMat* imageStream;
  if(1 == image.channels())
  {
	cout << "Converting from single channel to triple channel image." << endl;
	cv::Mat convertedImage;
	cv::cvtColor( image, convertedImage, CV_GRAY2RGB );
	imageStream = BitmapCreator::CreateBitmapFromMat(convertedImage);
  }
  else
  {
	imageStream = BitmapCreator::CreateBitmapFromMat(image);
  }

  bool imageLoaded   = Commander->LCR_LOAD_STATIC_IMAGE( imageStream->data.ptr, imageStream->step );

  if(!imageLoaded)
	{ cout<<"Could not load static 608 x 684 24bit static bmp.\n"; }
  else
	{ cout<<"Static Image Loaded.\n"; }
	 
  return imageLoaded;
}