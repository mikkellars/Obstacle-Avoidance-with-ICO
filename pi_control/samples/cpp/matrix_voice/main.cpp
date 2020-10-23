#include <unistd.h>
#include <iostream>
#include <cmath>

#include <matrix_hal/everloop.h>
#include <matrix_hal/everloop_image.h>
#include <matrix_hal/gpio_control.h>
#include <matrix_hal/matrixio_bus.h>

// GPIO via Matrix Voice
#define  TB6612_RIGHT_MOTOR_PWMA        14 // (Orange)
#define  TB6612_LEFT_MOTOR_PWMB         8  // (Green)
#define  TB6612_RIGHT_MOTOR_AIN1        12 // (Blue)
#define  TB6612_RIGHT_MOTOR_AIN2        10 // (Brown)
#define  TB6612_LEFT_MOTOR_BIN1         6  // (Grey)
#define  TB6612_LEFT_MOTOR_BIN2         4  // (Pink)

// Matrix Voice LEDs
#define MATRIX_LED_R_1		0
#define MATRIX_LED_R_2		1
#define MATRIX_LED_R_3		2
#define MATRIX_LED_R_4		3
#define MATRIX_LED_R_5		4
#define MATRIX_LED_R_6		5
#define MATRIX_LED_R_7		6
#define MATRIX_LED_R_8		7
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


using namespace std;

int main (int argc, char** argv)
{
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
	cout << "Running LED animation..." << endl;
	startupShowLEDRainbow(&everloop, &everloop_image);

        // Initialise Matrix Voice GPIO pins
        initGPIOPins(&gpio);

	cout << "Turning on motor..." << endl;
	int motorR = 25, motorL = 25;

	// Rotate left
	setRightMotorSpeedDirection(&gpio,motorR,1);
        setLeftMotorSpeedDirection(&gpio,motorL,0);

	setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_5, (int) (motorR*((float)255/255)), (int) (motorR*((float)69/255)), 0);
	setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_5, (int) (motorL*((float)255/255)), (int) (motorL*((float)69/255)), 0);

	usleep(1000000);

	// Rotate right
	setRightMotorSpeedDirection(&gpio,motorR,0);
        setLeftMotorSpeedDirection(&gpio,motorL,1);

	setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_5, (int) (motorR*((float)255/255)), (int) (motorR*((float)69/255)), 0);
	setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_5, (int) (motorL*((float)255/255)), (int) (motorL*((float)69/255)), 0);

	usleep(1000000);

	cout << "Turning off motor..." << endl;

	setRightMotorSpeedDirection(&gpio,0,1);
        setLeftMotorSpeedDirection(&gpio,0,1);

	setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_R_5, 0, 0, 0);
	setMatrixVoiceLED(&everloop, &everloop_image, MATRIX_LED_L_5, 0, 0, 0);

	return 0;
}
