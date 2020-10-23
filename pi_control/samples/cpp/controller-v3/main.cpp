#include <cmath>
#include <ctime>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <raspicam/raspicam.h>
//#include <pigpio.h>
#include <thread>
#include <chrono>
#include <math.h>
#include <vector>
#include <string>
#include <utility>
#include <stdexcept>
#include <sstream>

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
int isPaused = 1;
int obstacleDistThreshold = 130;


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


void write_csv(std::string filename, std::vector<std::pair<std::string, std::vector<float>>> dataset) {
    // Make a CSV file with one or more columns of integer values
    // Each column of data is represented by the pair <column name, column data>
    //   as std::pair<std::string, std::vector<int>>
    // The dataset is represented as a vector of these columns
    // Note that all columns should be the same size

    // Create an output filestream object
    std::ofstream myFile(filename);

    size_t longestCol = 0;

    // Send column names to the stream
    for (size_t j = 0; j < dataset.size(); ++j)
    {
        if (dataset.at(j).second.size() > longestCol)
        {
            longestCol = dataset.at(j).second.size();
        }

        myFile << dataset.at(j).first;
        if (j != dataset.size() - 1) myFile << ","; // No comma at end of line
    }
    myFile << "\n";

    // Send data to the stream
    for (size_t i = 0; i < longestCol; ++i)
    {
        for (size_t j = 0; j < dataset.size(); ++j)
        {
            if (dataset.at(j).second.size() > i)
            {
                myFile << dataset.at(j).second.at(i);
            }
            else
            {
                myFile << "0";
            }
            if (j != dataset.size() - 1) myFile << ","; // No comma at end of line
        }
        myFile << "\n";
    }

    // Close the file
    myFile.close();
}

std::vector<std::pair<std::string, std::vector<float>>> read_csv(std::string filename) {
    // Reads a CSV file into a vector of <string, vector<int>> pairs where
    // each pair represents <column name, column values>

    // Create a vector of <string, int vector> pairs to store the result
    std::vector<std::pair<std::string, std::vector<float>>> result;

    // Create an input filestream
    std::ifstream myFile(filename);

    // Make sure the file is open
    if (!myFile.is_open())
    {
        ofstream newFile(filename);
        newFile.close();
        return result;
    }

    // Helper vars
    std::string line, colname;
    float val;

    // Read the column names
    if (myFile.good())
    {
        // Extract the first line in the file
        std::getline(myFile, line);

        // Create a stringstream from line
        std::stringstream ss(line);

        // Extract each column name
        while (std::getline(ss, colname, ',')) {

            // Initialize and add <colname, int vector> pairs to result
            result.push_back({ colname, std::vector<float> {} });
        }
    }

    // Read data, line by line
    while (std::getline(myFile, line))
    {
        // Create a stringstream of the current line
        std::stringstream ss(line);

        // Keep track of the current column index
        int colIdx = 0;

        // Extract each float
        while (ss >> val) {

            // Add the current integer to the 'colIdx' column's values vector
            result.at(colIdx).second.push_back(val);

            // If the next token is a comma, ignore it and move on
            if (ss.peek() == ',') ss.ignore();

            // Increment the column index
            colIdx++;
        }
    }

    // Close file
    myFile.close();

    return result;
}

/****************************************************************************/
enum states
{
	paused,
	searching,
	follow,
	targetLost,
	obstacle
} state = searching;


bool timerComplete;
bool timerRunning;

void _timer(int x)
{
	this_thread::sleep_for(chrono::seconds(x));
	timerComplete = 1;
	timerRunning = 0;
}

void timer(int x)
{
	if(timerRunning) return;
	timerComplete = 0;
	timerRunning = 1;
	
	thread _timerthread(_timer, x);
	_timerthread.detach();
}



/*********************************   DONE   *********************************/

float activationFunction(float value, float shift)
{
	return 1/(1+shift*exp(-value));
}

vector<float> weights = { 0, 1 };
vector<float> hsvR = { 155, 179, 44, 168, 74, 202 };
vector<float> hsvB = { 0, 141, 62, 255, 0, 255 };
vector<float> controller = { 1, 600 };
vector<pair<string, vector<float>>> stored_data;

/******** Timer thread **************/
void timerCount(bool &check)
{
	while(true)
	{
		if(check == false)
		{
			this_thread::sleep_for(chrono::milliseconds(2000));
			check = true;	
		}
		this_thread::sleep_for(chrono::milliseconds(10));
	}
}


int main (int argc, char** argv)
{
	stored_data = read_csv("data.csv");

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
	vector<cv::KeyPoint> keyptsR;
	vector<cv::KeyPoint> keyptsB;

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
 	int iLowHR = 155;
 	int iHighHR = 179;

	int iLowSR = 44;
 	int iHighSR = 168;

 	int iLowVR = 74;
 	int iHighVR = 202;
	
	if (!stored_data.empty())
    {
        if (stored_data.size() >= 2 && stored_data.at(1).second.size() >= 6)
        {
            iLowHR = stored_data.at(1).second.at(0);
            iHighHR = stored_data.at(1).second.at(1);

            iLowSR = stored_data.at(1).second.at(2);
            iHighSR = stored_data.at(1).second.at(3);

            iLowVR = stored_data.at(1).second.at(4);
            iHighVR = stored_data.at(1).second.at(5);
        }
    }

	// Create a window for displaying HSV
	cv::namedWindow("HSV controls R",cv::WINDOW_NORMAL);

	// Create trackbars for H, S and V in the window
	cv::createTrackbar("LowH", "HSV controls R", &iLowHR, 179); //Hue (0 - 179)
 	cv::createTrackbar("HighH", "HSV controls R", &iHighHR, 179);

 	cv::createTrackbar("LowS", "HSV controls R", &iLowSR, 255); //Saturation (0 - 255)
 	cv::createTrackbar("HighS", "HSV controls R", &iHighSR, 255);

 	cv::createTrackbar("LowV", "HSV controls R", &iLowVR, 255); //Value (0 - 255)
 	cv::createTrackbar("HighV", "HSV controls R", &iHighVR, 255);

	// Create a bunch of windows for displaying image processing steps
	// Create window to display original HSV image
	cv::namedWindow("HSV image",cv::WINDOW_AUTOSIZE);
	// Create window to display thresholded image
	//cv::namedWindow("Thresholded image R",cv::WINDOW_AUTOSIZE);
	// Create window to display blob image
	cv::namedWindow("Blobs R",cv::WINDOW_AUTOSIZE);

	// Set minimum and maximum values for hue, saturation and value (HSV colourspace)
 	int iLowHB = 0;
 	int iHighHB = 141;

	int iLowSB = 62;
 	int iHighSB = 255;

 	int iLowVB = 0;
 	int iHighVB = 255;
	
	if (!stored_data.empty())
    {
        if (stored_data.size() >= 3 && stored_data.at(2).second.size() >= 6)
        {
            iLowHB = stored_data.at(2).second.at(0);
            iHighHB = stored_data.at(2).second.at(1);

            iLowSB = stored_data.at(2).second.at(2);
            iHighSB = stored_data.at(2).second.at(3);

            iLowVB = stored_data.at(2).second.at(4);
            iHighVB = stored_data.at(2).second.at(5);
        }
    }

	// Create a window for displaying HSV
	cv::namedWindow("HSV controls B",cv::WINDOW_NORMAL);

	// Create trackbars for H, S and V in the window
	cv::createTrackbar("LowH", "HSV controls B", &iLowHB, 179); //Hue (0 - 179)
 	cv::createTrackbar("HighH", "HSV controls B", &iHighHB, 179);

 	cv::createTrackbar("LowS", "HSV controls B", &iLowSB, 255); //Saturation (0 - 255)
 	cv::createTrackbar("HighS", "HSV controls B", &iHighSB, 255);

 	cv::createTrackbar("LowV", "HSV controls B", &iLowVB, 255); //Value (0 - 255)
 	cv::createTrackbar("HighV", "HSV controls B", &iHighVB, 255);

	// Create a bunch of windows for displaying image processing steps
	// Create window to display thresholded image
	//cv::namedWindow("Thresholded image B",cv::WINDOW_AUTOSIZE);
	// Create window to display blob image
	cv::namedWindow("Blobs B",cv::WINDOW_AUTOSIZE);
	
	cv::namedWindow("Controller",cv::WINDOW_NORMAL);
	
	if (!stored_data.empty())
    {
        if (stored_data.size() >= 4 && stored_data.at(3).second.size() >= 2)
        {
            isPaused = stored_data.at(3).second.at(0);
            obstacleDistThreshold = stored_data.at(3).second.at(1);
        }
    }
	
	cv::createTrackbar("Paused", "Controller", &isPaused, 1); // Pause switch
	cv::createTrackbar("Obstacle Dist", "Controller", &obstacleDistThreshold, 600); // Pause switch

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
	cv::Mat imageThresholdR;
	cv::Mat imageThresholdB;

	// OpenCV image Mat to store image with detected blobs
	cv::Mat imageKeypointsR;
	cv::Mat imageKeypointsB;

	// Vector storing [x,y] co-ordinates of detected blobs
	vector<cv::Point2f> keyptXYR;
	vector<cv::Point2f> keyptXYB;
	
	
	/********* LOAD STORED DATA **********/
	stored_data = read_csv("data.csv");
	
	//stored_data = {{"Weights", weights}};
	//write_csv("data.csv", stored_data);
	
	if(!stored_data.empty() && stored_data.front().second.size() >= weights.size())
	{
		weights = stored_data.front().second;
	}


	bool count = true;
	thread counter(timer, count);
	counter.detach();
	int countDown = 10;
	int dir = 1;
	float leftWhellWeight = 1;
	float rightWhellWeight = 1;
	float leftShift = 1;
	float rightShift = 1;
	

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
		cv::inRange(imageMat,cv::Scalar(iLowHR,iLowSR,iLowVR),cv::Scalar(iHighHR,iHighSR,iHighVR),imageThresholdR);
		cv::inRange(imageMat,cv::Scalar(iLowHB,iLowSB,iLowVB),cv::Scalar(iHighHB,iHighSB,iHighVB),imageThresholdB);

		//morphological opening (remove small objects from the foreground)
		erode(imageThresholdR, imageThresholdR, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
		dilate(imageThresholdB, imageThresholdB, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

		//morphological closing (fill small holes in the foreground)
		dilate(imageThresholdR, imageThresholdR, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
	 	erode(imageThresholdB, imageThresholdB, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

		// Display HSV image
		cv::imshow("HSV image", imageMat);

		// Display thresholded imnage
		//cv::imshow("Thresholded image R",imageThresholdR);
		//cv::imshow("Thresholded image B",imageThresholdB);

		// Detect keypoints in thresholded image using SimpleBlobDetector object
		sbd->detect(imageThresholdR,keyptsR);
		sbd->detect(imageThresholdB,keyptsB);

		// Draw detected keypoints as red circles around detected blobs and store new image in OpenCV image Mat
		cv::drawKeypoints(imageThresholdR,keyptsR,imageKeypointsR,cv::Scalar(0,0,255),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		cv::drawKeypoints(imageThresholdB,keyptsB,imageKeypointsB,cv::Scalar(0,0,255),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

		// Display image with detected blobs
		cv::imshow("Blobs R",imageKeypointsR);
		cv::imshow("Blobs B",imageKeypointsB);

		// Extract [x,y] co-ordinates of blobs from keypoints
		cv::KeyPoint::convert(keyptsR,keyptXYR);
		cv::KeyPoint::convert(keyptsB,keyptXYB);

		// If at least one blob has been detected (set to 1 here), print out the [x,y] co-ordinates
		//if (keyptXY.size() > 1)
		//cout << "[x,y] = " << "[" << keyptXY.front().x << "," << keyptXY.front().y << "]" << endl;

		/**************************
		*** YOUR CODE GOES HERE ***
		**************************/
		
		// Variables for direction
		float right, left;
		bool searchDirection = 0;					//0 = left, right = 1
		
		float targetPosX;
		float targetPosY;
		vector<float> targetDist = { 0 } ;
		
		float obstaclePosX;
		float obstaclePosY;
		vector<float> obstacleDist = { 0 } ;
		
		float deltaWeight = 0;
		float vLearning = 0;
		

		
		// Look for target
		if(keyptXYR.size() >= 1)
		{
			// Determine variables
			targetPosX = keyptXYR.front().x;
			targetPosY = keyptXYR.front().y;
			targetDist.push_back(keyptsR.front().size);	
			
			//Follow target
			right = (keyptXYR.front().x/Camera.getWidth())*1;		//Calculate relative position, between 0-1
			left = 1 - right;						//for left and right side.	
			
			if (right > 0.70)						//Clamping, ensure speed limit
			{
				right = 0.70;
			}
			
			if (left > 0.70)
			{
				left = 0.70;
			}	
			countDown = 10;
		}
		else if(keyptXYR.size() < 1)						// If no target is found
		{
			if(countDown != 0)
			{
				this_thread::sleep_for(chrono::milliseconds(100));
				countDown -= 1;
				right = 0.42f;							// Speed to ensure 25 speed
				left = 0.42f;
			}
			else
			{
				
				//Improve this! Currently, if it can't find target, it will move in cirkels 
				if(dir == -1)
				{
					right = 0.5;
					left = -5;
				}
				else
				{
					right = -5;
					left = 0.5;
				}
			}
		}
		
		// Look for obstacle		
		if(keyptXYB.size() >= 1)
		{
			int obstIndex = 0;
			for(size_t i = 0; i < keyptsB.size(); i++)
			{
				if(keyptsB.at(i).size > keyptsB.at(obstIndex).size) obstIndex = i;
			}
			obstaclePosX = keyptXYB.at(obstIndex).x;
			obstaclePosY = keyptXYB.at(obstIndex).y;
			obstacleDist.push_back(keyptsB.at(obstIndex).size);
			
			if(obstacleDist[obstacleDist.size()-1] > obstacleDistThreshold)	// If target is within thresshold
			{
				// Learning obstacle avoidance for rightWhell
				float mu = 0.1;																//Learning algorithm
				deltaWeight = pow(mu * obstacleDist[obstacleDist.size()-1] * ((obstacleDist[obstacleDist.size()-2]-obstacleDist[obstacleDist.size()-3])), 47);
				cout << "DeltaWeight: " << deltaWeight << endl;
				weights.at(0) = weights.at(0) + deltaWeight;
				dir = obstaclePosX / Camera.getWidth() > 0.5 ? -1 : 1;
				vLearning = ((obstacleDist[obstacleDist.size() - 1] * weights.at(0)) + (obstacleDist[obstacleDist.size() - 2] * weights.at(1))) * dir;
				  cout << "vLearning :" << vLearning << endl;
				
				/*
				// Shift activation functions
				if(obstaclePosX <= Camera.getWidth()/2)
				{
				leftWhellWeight = leftWhellWeight + (targetDist[targetDist.size()-1] * (obstacleDist[obstacleDist.size()-2]-obstacleDist[obstacleDist.size()-3]));			// Calculate weights
				leftShift = targetDist[targetDist.size() - 1] * leftWhellWeight + obstacleDist[obstacleDist.size()];
				cout << "Left shift : " << leftShift << endl;
				}
				else if(obstaclePosX > Camera.getWidth()/2)
				{
				rightWhellWeight = rightWhellWeight + (targetDist[targetDist.size()-1] * (obstacleDist[obstacleDist.size()-2]-obstacleDist[obstacleDist.size()-3]));			// Calculate weights
				rightShift = targetDist[targetDist.size() - 1] * rightWhellWeight + obstacleDist[obstacleDist.size()];
				cout << "Right shift : " << rightShift << endl;
				}
				* */
			}
		}
		
		// Set motor speeds
		int motorR = 60 * activationFunction(left, leftShift), motorL = (60 * activationFunction(right, rightShift)) + vLearning;
		//cout << weights.at(0) << endl;
		cout << "Left shift: " << leftShift << ": Right Shift: " << rightShift << endl;
		//cout << motorL << " : " << motorR << endl;
		
		// Print out debug information
		if(keyptXYB.size() >= 1)
		{
			//cout << "Target [ " << targetPosX << " , " << targetPosY << " , " << targetDist[targetDist.size()-1] << " ], Obstacle  [ " << obstaclePosX << " , " << obstaclePosY << " , " << obstacleDist[obstacleDist.size()-1] << " ], State [" << state << "]" << endl;
		}
		
		//Top LED Control
		if(keyptXYR.size() < 1)
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



		if(keyptXYB.size() < 1)
		{
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_1, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_9, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_2, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_8, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_3, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_7, 0, 0, 0);
		}
		else if ((obstaclePosX < Camera.getWidth()*0.60 && obstaclePosX > Camera.getWidth() * 0.40))
		{
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_1, 45, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_9, 45, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_2, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_8, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_3, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_7, 0, 0, 0);
		}
		else if (obstaclePosX > Camera.getWidth() * 0.60 && obstaclePosX <= Camera.getWidth() * 0.80)
		{
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_1, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_9, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_2, 45, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_8, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_3, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_7, 0, 0, 0);
		}
		else if (obstaclePosX < Camera.getWidth() * 0.40 && obstaclePosX > Camera.getWidth() * 0.3)
		{
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_1, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_9, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_2, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_8, 45, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_3, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_7, 0, 0, 0);
		}
		else if (obstaclePosX > Camera.getWidth() * 0.8)
		{
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_1, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_9, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_2, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_8, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_3, 45, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_7, 0, 0, 0);
		}
		else if (obstaclePosX < Camera.getWidth() * 0.3)
		{
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_1, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_9, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_2, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_8, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_3, 0, 0, 0);
			setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_7, 45, 0, 0);
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


	        setRightMotorSpeedDirection(&gpio, motorR*!isPaused, 1);
        	setLeftMotorSpeedDirection(&gpio, motorL*!isPaused, 1);

	        setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_5, (int) (motorR*((float)255/255)), (int) (motorR*((float)69/255)), 0);
        	setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_5, (int) (motorL*((float)255/255)), (int) (motorL*((float)69/255)), 0);

		// Wait 30ms for keypress and exit if ESC (ASCII code 27) is pressed
		if (cv::waitKey(30) == 27)
			break;
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
	
	/********** SAVE STORED WEIGHTS **********/

    hsvR = { (float)iLowHR, (float)iHighHR, (float)iLowSR, (float)iHighSR, (float)iLowVR, (float)iHighVR };
    hsvB = { (float)iLowHB, (float)iHighHB, (float)iLowSB, (float)iHighSB, (float)iLowVB, (float)iHighVB };
    controller = { (float)isPaused, (float)obstacleDistThreshold };

    stored_data = { {"Weights", weights}, {"HSV_R", hsvR}, {"HSV_B", hsvB}, {"Controller", controller} };
    write_csv("data.csv", stored_data);
    
    cout << "Weight: " << weights.at(0) << endl;

	// Release camera resources
	Camera.release();

	return 0;
}
