#include <cmath>
#include <ctime>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <raspicam/raspicam.h>
//#include <pigpio.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

#include <matrix_hal/everloop.h>
#include <matrix_hal/everloop_image.h>
#include <matrix_hal/gpio_control.h>
#include <matrix_hal/matrixio_bus.h>


/* Without Matrix Voice
#define TB6612_RIGHT_MOTOR_PWMA		12 // (Orange)
#define TB6612_LEFT_MOTOR_PWMB		13 // (Green)
#define TB6612_RIGHT_MOTOR_AIN1		16 // (Blue)
#define TB6612_RIGHT_MOTOR_AIN2		26 // (Brown)
#define TB6612_LEFT_MOTOR_BIN1		5  // (Grey)
#define TB6612_LEFT_MOTOR_BIN2		6  // (Pink)


// With Matrix Voice but using Raspberry Pi GPIO
#define  TB6612_RIGHT_MOTOR_PWMA        13 // (Orange)
#define  TB6612_LEFT_MOTOR_PWMB         16 // (Green)
#define  TB6612_RIGHT_MOTOR_AIN1        19 // (Blue)
#define  TB6612_RIGHT_MOTOR_AIN2        26 // (Brown)
#define  TB6612_LEFT_MOTOR_BIN1         20 // (Grey)
#define  TB6612_LEFT_MOTOR_BIN2         21 // (Pink)
*/

// GPIO via Matrix Voice
#define  TB6612_RIGHT_MOTOR_PWMA        14 // (Orange)
#define  TB6612_LEFT_MOTOR_PWMB         8  // (Green)
#define  TB6612_RIGHT_MOTOR_AIN1        12 // (Blue)
#define  TB6612_RIGHT_MOTOR_AIN2        10 // (Brown)
#define  TB6612_LEFT_MOTOR_BIN1         6  // (Grey)
#define  TB6612_LEFT_MOTOR_BIN2         4  // (Pink)

// Matrix Voice LEDs
#define MATRIX_LED_R_1          0
#define MATRIX_LED_R_2          1
#define MATRIX_LED_R_3          2
#define MATRIX_LED_R_4          3
#define MATRIX_LED_R_5          4
#define MATRIX_LED_R_6          5
#define MATRIX_LED_R_7          6
#define MATRIX_LED_R_8          7
#define MATRIX_LED_R_9          8

#define MATRIX_LED_L_1          9
#define MATRIX_LED_L_2          10
#define MATRIX_LED_L_3          11
#define MATRIX_LED_L_4          12
#define MATRIX_LED_L_5          13
#define MATRIX_LED_L_6          14
#define MATRIX_LED_L_7          15
#define MATRIX_LED_L_8          16
#define MATRIX_LED_L_9          17

bool following = 0;
bool paused = 0;


using namespace std;
//using namespace cv;

// Initialise Matrix Voice GPIO pins
void initGPIOPins(matrix_hal::GPIOControl* gpio)
{

        gpio->SetMode(TB6612_RIGHT_MOTOR_PWMA,1); //Pin mode as output
        gpio->SetFunction(TB6612_RIGHT_MOTOR_PWMA,1); // Pin function as PWM
        gpio->SetMode(TB6612_RIGHT_MOTOR_AIN1,1);
        gpio->SetMode(TB6612_RIGHT_MOTOR_AIN2,1);

        gpio->SetMode(TB6612_LEFT_MOTOR_PWMB,1); //Pin mode as output
        gpio->SetFunction(TB6612_LEFT_MOTOR_PWMB,1); // Pin function as PWM
        gpio->SetMode(TB6612_LEFT_MOTOR_BIN1,1);
        gpio->SetMode(TB6612_LEFT_MOTOR_BIN2,1);
}

// Set speed and direction of LEFT motor
// Directiom -> 1 = forward, 0 = reverse
// Speed -> 0-100% in steps of 1%
void setLeftMotorSpeedDirection(matrix_hal::GPIOControl* gpio, int speed, int dir)
{
        if (dir <= 0) // Reverse
        {
                gpio->SetGPIOValue(TB6612_LEFT_MOTOR_BIN1,0); // Rotate left motor clockwise
                gpio->SetGPIOValue(TB6612_LEFT_MOTOR_BIN2,1);
        }
        if ( (dir > 0)  || (dir >= 1) ) // Forward
        {
                gpio->SetGPIOValue(TB6612_LEFT_MOTOR_BIN1,1); // Rotate left motor counter-clockwise
                gpio->SetGPIOValue(TB6612_LEFT_MOTOR_BIN2,0);
        }

        // Set motor speed via PWM signal (min. = 0, max. = 100)
       if (speed > 100)
                speed = 100;
        if (speed < 0)
                speed = 0;

        gpio->SetPWM(1000,speed,TB6612_LEFT_MOTOR_PWMB);
}

// Set speed and direction of RIGHT motor
// Directiom -> 1 = forward, 0 = reverse
// Speed -> 0-100% in steps of 1%
void setRightMotorSpeedDirection(matrix_hal::GPIOControl* gpio, int speed, int dir)
{
        if (dir <= 0) // Reverse
        {
                gpio->SetGPIOValue(TB6612_RIGHT_MOTOR_AIN1,0); // Rotate right motor counter-clockwise
                gpio->SetGPIOValue(TB6612_RIGHT_MOTOR_AIN2,1);
        }
        if ( (dir > 0)  || (dir >= 1) ) // Forward
        {
                gpio->SetGPIOValue(TB6612_RIGHT_MOTOR_AIN1,1); // Rotate right motor clockwise
                gpio->SetGPIOValue(TB6612_RIGHT_MOTOR_AIN2,0);
        }

        // Set motor speed via PWM signal (min. = 0, max. = 100)
        if (speed > 100)
                speed = 100;
        if (speed < 0)
                speed = 0;

        gpio->SetPWM(1000,speed,TB6612_RIGHT_MOTOR_PWMA);
}

// Display a rainbow animation on the Matrix Voice LEDs
void startupShowLEDRainbow(matrix_hal::Everloop* everloop, matrix_hal::EverloopImage* everloop_image)
{
        // Variables used for sine wave rainbow logic
        float counter = 0;
        const float freq = 0.375;

        // For each led in everloop_image.leds, set led value
        for (matrix_hal::LedValue &led : everloop_image->leds)
        {
                // Sine waves 120 degrees out of phase for rainbow
                led.red = (std::sin(freq * counter + (M_PI / 180 * 240)) * 155 + 100) / 10;
                led.green = (std::sin(freq * counter + (M_PI / 180 * 120)) * 155 + 100) / 10;
                led.blue = (std::sin(freq * counter + 0) * 155 + 100) / 10;
                counter = counter + 1.01;
                // Updates the LEDs
                everloop->Write(everloop_image);
                usleep(40000);
        }

        usleep(460000);

        // For each led in everloop_image.leds, set led value to 0
        for (matrix_hal::LedValue &led : everloop_image->leds)
        {
                // Turn off Everloop
                led.red = 0;
                led.green = 0;
                led.blue = 0;
                led.white = 0;
        }

        // Updates the Everloop on the MATRIX device
        everloop->Write(everloop_image);
}

// Set individual LEDs on the Matrix Voice
void setMatrixVoiceLED(matrix_hal::Everloop* everloop, matrix_hal::EverloopImage* everloop_image,int ledn, int r, int g, int b)
{
       for (int i = 0; i < 18; i++)
        {
                if (i == ledn)
                {
                        everloop_image->leds[ledn].red = r;
                        everloop_image->leds[ledn].green = g;
                        everloop_image->leds[ledn].blue = b;
                        everloop_image->leds[ledn].white = 0;
                }
        }
        everloop->Write(everloop_image);
}


int main (int argc, char** argv)
{

/*****************************************************************************
*********************   INITIALISE MATRIX VOICE DEVICE   *********************
*****************************************************************************/

        // Create MatrixIOBus object for hardware communication
        matrix_hal::MatrixIOBus bus;
        // Initialize bus and exit program if error occurs
        if (!bus.Init())
		return false;

        // Holds the number of LEDs on MATRIX device
        int ledCount = bus.MatrixLeds();
        // Create EverloopImage object, with size of ledCount
        matrix_hal::EverloopImage everloop_image(ledCount);
        // Create Everloop object
        matrix_hal::Everloop everloop;
        // Set everloop to use MatrixIOBus bus
        everloop.Setup(&bus);
        // Create GPIOControl object
        matrix_hal::GPIOControl gpio;
        // Set gpio to use MatrixIOBus bus
        gpio.Setup(&bus);

	// Display rainbow animation
	startupShowLEDRainbow(&everloop, &everloop_image);

/*********************************   DONE   *********************************/


/*****************************************************************************
************************   INITIALISE MOTOR CONTROL   ************************
*****************************************************************************/

	// Initialise Matrix Voice GPIO pins
        initGPIOPins(&gpio);

/*********************************   DONE   *********************************/


/*****************************************************************************
*********************   INITIALISE RASPBERRY PI CAMERA   *********************
*****************************************************************************/

	// Create Raspberry Pi camera object
	raspicam::RaspiCam Camera;
	// Create OpenCV SimpleBlobDetector parameter object
	cv::SimpleBlobDetector::Params sbdPar;

	// Set blob detection parameters
	// Change thresholds
	sbdPar.minThreshold = 1;
	sbdPar.maxThreshold = 1000;
	// Filter by colour
	sbdPar.filterByColor = true;
	// Look for colours that match grayscale value of 255 (white)
	sbdPar.blobColor=255;
	// Filter by area
	sbdPar.filterByArea = true;
	sbdPar.minArea = 35; // 100x100 pixels
	sbdPar.maxArea = 160000; // 400x400 pixels

	// Create OpenCV SimpleBlobDetector object based on assigned parameters
	cv::Ptr<cv::SimpleBlobDetector> sbd = cv::SimpleBlobDetector::create(sbdPar);
	vector<cv::KeyPoint> keypts;

	// Set Raspberry Pi camera parameters
	// Set camera image format to BGR as used by  OpenCV
	Camera.setFormat(raspicam::RASPICAM_FORMAT_BGR);
	// Set image resolution
	Camera.setWidth(640);
	Camera.setHeight(480);
        // Flip camera image vertically and horizontally
        // because camera is mounted upside down
        Camera.setVerticalFlip(true);
        Camera.setHorizontalFlip(true);

	// Display current camera parameters
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
	cout << "Format:" << Camera.getFormat() << endl;

	// Open camera
	if (!Camera.open())
	{
		cerr << "Error opening camera." << endl;
		return -1;
	}

	// Wait 3 seconds for camera image to stabilise
	cout << "Waiting for camera to stabilise...";
	usleep(3000000);
	cout << "done." << endl;

/*********************************   DONE   *********************************/


/*****************************************************************************
************************   INITIALISE VISUALISATION   ************************
*****************************************************************************/

	// Set minimum and maximum values for hue, saturation and value (HSV colourspace)
 	int iLowH = 155;
 	int iHighH = 179;

	int iLowS = 44;
 	int iHighS = 168;

 	int iLowV = 74;
 	int iHighV = 202;

	// Create a window for displaying HSV
	cv::namedWindow("HSV controls",cv::WINDOW_NORMAL);

	// Create trackbars for H, S and V in the window
	cv::createTrackbar("LowH", "HSV controls", &iLowH, 179); //Hue (0 - 179)
 	cv::createTrackbar("HighH", "HSV controls", &iHighH, 179);

 	cv::createTrackbar("LowS", "HSV controls", &iLowS, 255); //Saturation (0 - 255)
 	cv::createTrackbar("HighS", "HSV controls", &iHighS, 255);

 	cv::createTrackbar("LowV", "HSV controls", &iLowV, 255); //Value (0 - 255)
 	cv::createTrackbar("HighV", "HSV controls", &iHighV, 255);

	// Create a bunch of windows for displaying image processing steps
	// Create window to display original HSV image
	cv::namedWindow("RGB image",cv::WINDOW_AUTOSIZE);
	// Create window to display thresholded image
	cv::namedWindow("Thresholded image",cv::WINDOW_AUTOSIZE);
	// Create window to display blob image
	cv::namedWindow("Blobs",cv::WINDOW_AUTOSIZE);

/*********************************   DONE   *********************************/


/*****************************************************************************
*******************************   CONTROLLER   *******************************
*****************************************************************************/

	// Create buffer of correct size to store image data
	int img_buf_len = Camera.getImageTypeSize(raspicam::RASPICAM_FORMAT_BGR);
	unsigned char *img_buf=new unsigned char[img_buf_len];

	// Initialise OpenCV image Mat
	cv::Mat imageMat = cv::Mat(Camera.getHeight(),Camera.getWidth(),CV_8UC3,img_buf);

	// OpeCV image Mat that holds the thresholded image data
	cv::Mat imageThreshold;

	// OpenCV image Mat to store image with detected blobs
	cv::Mat imageKeypoints;

	// Vector storing [x,y] co-ordinates of detected blobs
	vector<cv::Point2f> keyptXY;


	// Controller loop starts here
	while(true)
	{
		// Grab image into internal buffer
		Camera.grab();

		// Copy latest camera buffer into our defined buffer
		Camera.retrieve(img_buf);

		// Copy image buffer data into OpenCV Mat image
		imageMat = cv::Mat(Camera.getHeight(),Camera.getWidth(),CV_8UC3,img_buf);

		// Exit if there is no image data in OpenCV image Mat
		if (!imageMat.data)
		{
			cout << "No data in Mat imageMat." << endl;

			// Release Raspberry Pi camera resources
	                Camera.release();

			return -1;
		}

		// Convert image from BGR to HSV
		cv::cvtColor(imageMat,imageMat,cv::COLOR_BGR2HSV);

		// Threshold image
		cv::inRange(imageMat,cv::Scalar(iLowH,iLowS,iLowV),cv::Scalar(iHighH,iHighS,iHighV),imageThreshold);

		//morphological opening (remove small objects from the foreground)
		erode(imageThreshold, imageThreshold, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
		dilate(imageThreshold, imageThreshold, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

		//morphological closing (fill small holes in the foreground)
		dilate(imageThreshold, imageThreshold, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
	 	erode(imageThreshold, imageThreshold, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

		// Display HSV image
		cv::Mat imageMatRGB;
		cv::cvtColor(imageMat, imageMatRGB, cv::COLOR_HSV2BGR);
		cv::imshow("RGB image", imageMatRGB);

		// Display thresholded imnage
		cv::imshow("Thresholded image",imageThreshold);

		// Detect keypoints in thresholded image using SimpleBlobDetector object
		sbd->detect(imageThreshold,keypts);

		// Draw detected keypoints as red circles around detected blobs and store new image in OpenCV image Mat
		cv::drawKeypoints(imageThreshold,keypts,imageKeypoints,cv::Scalar(0,0,255),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

		// Display image with detected blobs
		cv::imshow("Blobs",imageKeypoints);

		// Extract [x,y] co-ordinates of blobs from keypoints
		cv::KeyPoint::convert(keypts,keyptXY);

		// If at least one blob has been detected (set to 1 here), print out the [x,y] co-ordinates
		//if (keyptXY.size() > 1)
			//cout << "[x,y] = " << "[" << keyptXY.front().x << "," << keyptXY.front().y << "]" << endl;

		/**************************
		*** YOUR CODE GOES HERE ***
		**************************/
		//Variables for direction
		float right = 0.5, left = 0.5;
		
		//Calculate motor direction if target is seen
		if (paused)
		{
			right = 0;
			left = 0;
		}
		else if (keyptXY.size() >= 1) 
		{
			right = (keyptXY.front().x/Camera.getWidth())*1;
			left = 1 - right;
			
			if (right > 0.75)
				right = 0.75;
			if (left > 0.75)
				left = 0.75;
			
			following = 1;
		}
		else if (following)
		{
			right = 0;
			left = 0;
			following = 0;
		}
		
		
		
		if(keyptXY.size() < 1)
		{
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_1, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_9, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_2, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_8, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_3, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_7, 0, 0, 0);
		}
		else if ((right < 0.55 && right > 0.3) && (left < 0.55 && left > 0.3))
		{
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_1, 0, 50, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_9, 0, 50, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_2, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_8, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_3, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_7, 0, 0, 0);
		}
		else if (right > 0.55 && right <= 0.7)
		{
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_1, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_9, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_2, 0, 50, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_8, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_3, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_7, 0, 0, 0);
		}
		else if (left > 0.55 && left <= 0.7)
		{
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_1, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_9, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_2, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_8, 0, 50, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_3, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_7, 0, 0, 0);
		}
		else if (right > 0.7)
		{
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_1, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_9, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_2, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_8, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_3, 0, 45, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_7, 0, 0, 0);
		}
		else if (left > 0.7)
		{
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_1, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_9, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_2, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_8, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_3, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_7, 0, 45, 0);
		}
		else
		{
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_1, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_9, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_2, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_8, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_3, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_7, 0, 0, 0);
		}

		// Set motor speeds
		int motorR = 60 * left, motorL = 60 * right;
		cout << "[motorR,motorL] = " << "[" << motorR << "," << motorL << "]" << endl;
	        setRightMotorSpeedDirection(&gpio, motorR, 1);
        	setLeftMotorSpeedDirection(&gpio, motorL, 1);

	        setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_5, (int) (motorR*((float)255/255)), (int) (motorR*((float)69/255)), 0);
        	setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_5, (int) (motorL*((float)255/255)), (int) (motorL*((float)69/255)), 0);

		// Wait 30ms for keypress and exit if ESC (ASCII code 27) is pressed
		if (cv::waitKey(30) == 27)
			break;
		if (cv::waitKey(30) == 32)
		{
			paused = !paused;
		}
	} // End of while loop

/*********************************   DONE   *********************************/

	// Stop all motors
        setRightMotorSpeedDirection(&gpio,0,1);
        setLeftMotorSpeedDirection(&gpio,0,1);

        setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_5, 0, 0, 0);
        setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_5, 0, 0, 0);
	
	setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_1, 0, 0, 0);
	setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_9, 0, 0, 0);
	setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_2, 0, 0, 0);
	setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_8, 0, 0, 0);
	setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_3, 0, 0, 0);
	setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_7, 0, 0, 0);

	// Release camera resources
	Camera.release();

	return 0;
}
