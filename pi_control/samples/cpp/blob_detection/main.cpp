#include <ctime>
#include <fstream>
#include <iostream>
#include <chrono>
#include <thread>
#include <raspicam/raspicam.h>
#include <unistd.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>


using namespace std;
//using namespace cv;

int main (int argc, char** argv)
{
	// Create Raspberry Pi camera object
	raspicam::RaspiCam Camera;

	cv::SimpleBlobDetector::Params sbdPar;

	// Change thresholds
	sbdPar.minThreshold = 1;
	sbdPar.maxThreshold = 1000;
	// Filter by colour
	sbdPar.filterByColor = true;
	// Look for colours that match grayscale value of 255 (white)
	sbdPar.blobColor=255;
	// Filter by area
	sbdPar.filterByArea = true;
	sbdPar.minArea = 10000; // 100x100 pixels
	sbdPar.maxArea = 160000; // 400x400 pixels

	cv::Ptr<cv::SimpleBlobDetector> sbd = cv::SimpleBlobDetector::create(sbdPar);
	vector<cv::KeyPoint> keypts;

	// Check camera parameters
        cout << "Format: " << Camera.getFormat() << endl;
        cout << "Width: " << Camera.getWidth() << endl;
        cout << "Height: " << Camera.getHeight() << endl;
        cout << "Brightness: " << Camera.getBrightness() << endl;
        cout << "Rotation: " << Camera.getRotation() << endl;
        cout << "ISO: " << Camera.getISO() << endl;
        cout << "Sharrpness: " << Camera.getSharpness() << endl;
        cout << "Contrast: " << Camera.getContrast() << endl;
        cout << "Saturation: " << Camera.getSaturation() << endl;
        cout << "ShutterSpeed: " << Camera.getShutterSpeed() << endl;
        cout << "Exopsure: " << Camera.getExposure() << endl;
        cout << "AWB: " << Camera.getAWB() << endl;
        cout << "Image effect: " << Camera.getImageEffect() << endl;
        cout << "Metering: " << Camera.getMetering() << endl;
	cout << "Format:" << Camera.getFormat();

	// Set camera image format to BGR as used by  OpenCV
	Camera.setFormat(raspicam::RASPICAM_FORMAT_BGR);

	cout << "...changed to 2 (BGR)" << endl;

	// Set image resolution
	Camera.setWidth(640);
	Camera.setHeight(480);

	// Flip camera image vertically and horizontally
	// because camera is mounted upside down
	Camera.setVerticalFlip(true);
	Camera.setHorizontalFlip(true);

	// Open camera
	if (!Camera.open())
	{
		cerr << "Error opening camera." << endl;
		return -1;
	}

	// Wait 3 seconds for camera image to stabilise
	cout << "Waiting for camera stabilisation...";
	usleep(3000000);
	cout << "done" << endl;

	// Create buffer of correct size to store image data
	int img_buf_len = Camera.getImageTypeSize(raspicam::RASPICAM_FORMAT_BGR);
	unsigned char *img_buf=new unsigned char[img_buf_len];

	// Set minimum and maximum values for hue, saturation and value
 	int iLowH = 0;
 	int iHighH = 179;

	int iLowS = 0;
 	int iHighS = 255;

 	int iLowV = 0;
 	int iHighV = 255;

	// Create a window
	cv::namedWindow("HSV controls",cv::WINDOW_NORMAL);

	// Create trackbars for H, S and V in the window
	cv::createTrackbar("LowH", "HSV controls", &iLowH, 179); //Hue (0 - 179)
 	cv::createTrackbar("HighH", "HSV controls", &iHighH, 179);

 	cv::createTrackbar("LowS", "HSV controls", &iLowS, 255); //Saturation (0 - 255)
 	cv::createTrackbar("HighS", "HSV controls", &iHighS, 255);

 	cv::createTrackbar("LowV", "HSV controls", &iLowV, 255); //Value (0 - 255)
 	cv::createTrackbar("HighV", "HSV controls", &iHighV, 255);

	while(true)
	{
	// Grab image into internal buffer
	Camera.grab();

	// Copy latest camera buffer into our buffer
	Camera.retrieve(img_buf);

	// Create OpenCV Mat image from image buffer
	cv::Mat imageMat = cv::Mat(Camera.getHeight(),Camera.getWidth(),CV_8UC3,img_buf);

	if (!imageMat.data)
	{
		cout << "No data in Mat imageMat." << endl;
		return -1;
	}

	// Convert image from BGR to HSV
	cv::cvtColor(imageMat,imageMat,cv::COLOR_BGR2HSV);

	// Create new image Mat that holds the thresholded image data
	cv::Mat imageThreshold;
	// Threshold image
	cv::inRange(imageMat,cv::Scalar(iLowH,iLowS,iLowV),cv::Scalar(iHighH,iHighS,iHighV),imageThreshold);

	//morphological opening (remove small objects from the foreground)
	erode(imageThreshold, imageThreshold, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
	dilate(imageThreshold, imageThreshold, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

	//morphological closing (fill small holes in the foreground)
	dilate(imageThreshold, imageThreshold, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
 	erode(imageThreshold, imageThreshold, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

	// Create window and display OpenCV Mat image
	cv::namedWindow("HSV image",cv::WINDOW_AUTOSIZE);
	cv::imshow("HSV image", imageMat);

	// Display thresholded image, window is automatically created
	cv::namedWindow("Thresholded image",cv::WINDOW_AUTOSIZE);
	cv::imshow("Thresholded image",imageThreshold);

	cv::Mat imageKeypoints;

	sbd->detect(imageThreshold,keypts);

	cv::drawKeypoints(imageThreshold,keypts,imageKeypoints,cv::Scalar(0,0,255),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

	cv::namedWindow("Blobs",cv::WINDOW_AUTOSIZE);
	cv::imshow("Blobs",imageKeypoints);

	vector<cv::Point2f> keyptXY;
	cv::KeyPoint::convert(keypts,keyptXY);

	if (keyptXY.size() == 1)
		cout << "[x,y] = " << "[" << keyptXY.front().x << "," << keyptXY.front().y << "]" << endl;

	if (cv::waitKey(30) == 27)
	{
		// Release camera resources
		Camera.release();
		break;
	}
	}

	return 0;
}
