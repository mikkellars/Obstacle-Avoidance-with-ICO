#include <iostream>
#include <unistd.h>
#include <matrix_hal/gpio_control.h>
#include <matrix_hal/matrixio_bus.h>

//#include <pigpio.h>
//#include <cmath>

/*// Without Matrix Voice
#define TB6612_RIGHT_MOTOR_PWMA         12 // (Orange)
#define TB6612_LEFT_MOTOR_PWMB          13 // (Green)
#define TB6612_RIGHT_MOTOR_AIN1         16 // (Blue)
#define TB6612_RIGHT_MOTOR_AIN2         26 // (Brown)
#define TB6612_LEFT_MOTOR_BIN1          5  // (Grey)
#define TB6612_LEFT_MOTOR_BIN2          6  // (Pink)
*/

/*// With Matrix Voice but using Raspberry Pi GPIO
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

using namespace std;

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
                gpio->SetGPIOValue(TB6612_LEFT_MOTOR_BIN1,1); // Rotate left motor clockwise
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
                gpio->SetGPIOValue(TB6612_RIGHT_MOTOR_AIN1,0); // Rotate left motor clockwise
                gpio->SetGPIOValue(TB6612_RIGHT_MOTOR_AIN2,1);
        }
        if ( (dir > 0)  || (dir >= 1) ) // Forward
        {
                gpio->SetGPIOValue(TB6612_RIGHT_MOTOR_AIN1,1); // Rotate left motor clockwise
                gpio->SetGPIOValue(TB6612_RIGHT_MOTOR_AIN2,0);
        }

        // Set motor speed via PWM signal (min. = 0, max. = 100)
        if (speed > 100)
                speed = 100;
        if (speed < 0)
                speed = 0;

	gpio->SetPWM(1000,speed,TB6612_RIGHT_MOTOR_PWMA);
}

/*
void initGPIOPins(void)
{
	// Set all pins as output
	cout << "Initialising GPIO pins..." << endl;
	cout << gpioSetMode(TB6612_RIGHT_MOTOR_AIN1,PI_OUTPUT) << endl;
	cout << gpioSetMode(TB6612_RIGHT_MOTOR_AIN2,PI_OUTPUT) << endl;
	cout << gpioSetMode(TB6612_LEFT_MOTOR_BIN1,PI_OUTPUT) << endl;
	cout << gpioSetMode(TB6612_LEFT_MOTOR_BIN2,PI_OUTPUT) << endl;
	cout << gpioSetMode(TB6612_RIGHT_MOTOR_PWMA,PI_OUTPUT) << endl;
	cout << gpioSetMode(TB6612_LEFT_MOTOR_PWMB,PI_OUTPUT) << endl;
	cout << "Done" << endl;
}

// Set speed and direction of LEFT motor
// Directiom -> 1 = forward, 0 = reverse
// Speed -> 0-100% in steps of 1%
void setLeftMotorSpeedDirection(int speed, int dir)
{

	if (dir <= 0)
	{
		cout << "Rev...";
		cout << gpioWrite(TB6612_LEFT_MOTOR_BIN1,0) << endl;	// Rotate left motor clockwise
		cout << gpioWrite(TB6612_LEFT_MOTOR_BIN2,1) << endl;
	}
	if ( (dir > 0)  || (dir >= 1) )
	{
		cout << "Fwd...";
		cout << gpioWrite(TB6612_LEFT_MOTOR_BIN1,1) << endl;	// Rotate left motor counter-clockwise
		cout << gpioWrite(TB6612_LEFT_MOTOR_BIN2,0) << endl;
	}

	int pwmDutyCycle = (int)floor((255*speed)/100);
	// Set motor speed via PWM signal (min. = 0, max. = 255)
	if (speed > 255)
		speed = 255;
	if (speed < 0)
		speed = 0;
	if (gpioPWM(TB6612_LEFT_MOTOR_PWMB,pwmDutyCycle) == 0)
		cout << "Left motor speed set to " << speed << "%" << endl;

}

// Set speed and direction of RIGHT motor
// Directiom -> 1 = forward, 0 = reverse
// Speed -> 0-100% in steps of 1%
void setRightMotorSpeedDirection(int speed, int dir)
{

	if (dir <= 0)
	{
		cout << "Rev...";
		cout << gpioWrite(TB6612_RIGHT_MOTOR_AIN1,0) << endl;	// Rotate right motor counter-clockwise
		cout << gpioWrite(TB6612_RIGHT_MOTOR_AIN2,1) << endl;
	}
	if ( (dir > 0)  || (dir >= 1) )
	{
		cout << "Fwd...";
		cout << gpioWrite(TB6612_RIGHT_MOTOR_AIN1,1) << endl;	// Rotate right motor clockwise
		cout << gpioWrite(TB6612_RIGHT_MOTOR_AIN2,0) << endl;
	}

	int pwmDutyCycle = (int)floor((255*speed)/100);
	// Set motor speed via PWM signal (min. = 0, max. = 255)
	if (speed > 255)
		speed = 255;
	if (speed < 0)
		speed = 0;
	if (gpioPWM(TB6612_RIGHT_MOTOR_PWMA,pwmDutyCycle) == 0)
		cout << "Right motor speed set to " << speed << "%" << endl;

}
*/

int main (int argc, char **argv)
{
	// Create MatrixIOBus object for hardware communication
        matrix_hal::MatrixIOBus bus;
        // Initialize bus and exit program if error occurs
        if (!bus.Init())
		return false;
	// Create GPIOControl object
	matrix_hal::GPIOControl gpio, old_gpio;
	// Set gpio to use MatrixIOBus bus
	gpio.Setup(&bus);

	// Initialise Matrix Voice GPIO pins
	initGPIOPins(&gpio);

	// Forward
	setRightMotorSpeedDirection(&gpio,25,1);
	setLeftMotorSpeedDirection(&gpio,25,1);

	usleep(1000000);

	// Reverse
	setRightMotorSpeedDirection(&gpio,25,0);
	setLeftMotorSpeedDirection(&gpio,25,0);

	usleep(1000000);

	// Turn left
	setRightMotorSpeedDirection(&gpio,25,1);
	setLeftMotorSpeedDirection(&gpio,25,0);

	usleep(1000000);

	// Turn right
	setRightMotorSpeedDirection(&gpio,25,0);
	setLeftMotorSpeedDirection(&gpio,25,1);

	usleep(1000000);

	// Stop
	setRightMotorSpeedDirection(&gpio,0,1);
	setLeftMotorSpeedDirection(&gpio,0,1);

/*	// Inilialise PIGPIO library
	if (gpioInitialise() < 0)
		cout << "PIGPIO library initialisation failed." << endl;

	// Initialise GPIO pins
	initGPIOPins();

	setRightMotorSpeedDirection(25,1);
	setLeftMotorSpeedDirection(25,1);

	usleep(5000000);

	setRightMotorSpeedDirection(25,0);
	setLeftMotorSpeedDirection(25,0);

	usleep(5000000);

	setRightMotorSpeedDirection(0,1);
	setLeftMotorSpeedDirection(0,1);

	// Terminate PIGPIO library and release resources
	gpioTerminate();
*/

	return 0;
}
