"""
Controller for the robot.
"""


from time import sleep, time
from math import pi, sin
from matrix_lite import gpio
from matrix_lite import led


# GPIO  via Matrix Voice
TB6612_RIGHT_MOTOR_PWMA = 14    # (Orange)
TB6612_LEFT_MOTOR_PWMB = 8      # (Green)
TB6612_RIGHT_MOTOR_AIN1 = 12    # (Blue)
TB6612_RIGHT_MOTOR_AIN2 = 10    # (Brown)
TB6612_LEFT_MOTOR_BIN1 = 6      # (Grey)
TB6612_LEFT_MOTOR_BIN2 = 4      # (Pink)

# Matrix Voice LEDs
MATRIX_LED_R_1 = 0
MATRIX_LED_R_2 = 1
MATRIX_LED_R_3 = 2
MATRIX_LED_R_4 = 3
MATRIX_LED_R_5 = 4
MATRIX_LED_R_6 = 5
MATRIX_LED_R_7 = 6
MATRIX_LED_R_8 = 7
MATRIX_LED_R_9 = 8

MATRIX_LED_L_1 = 9
MATRIX_LED_L_2 = 10
MATRIX_LED_L_3 = 11
MATRIX_LED_L_4 = 12
MATRIX_LED_L_5 = 13
MATRIX_LED_L_6 = 14
MATRIX_LED_L_7 = 15
MATRIX_LED_L_8 = 16
MATRIX_LED_L_9 = 17


def init_gpio_pins()->None:
    """Init GPIO pins.
    """
    # Right motor
    gpio.setMode(TB6612_RIGHT_MOTOR_PWMA, 'output')
    gpio.setFunction(TB6612_RIGHT_MOTOR_PWMA, 'PWM')
    gpio.setMode(TB6612_RIGHT_MOTOR_AIN1, 'output')
    gpio.setMode(TB6612_RIGHT_MOTOR_AIN2, 'output')

    # Left motor
    gpio.setMode(TB6612_LEFT_MOTOR_PWMB, 'output')
    gpio.setFunction(TB6612_LEFT_MOTOR_PWMB, 'PWM')
    gpio.setMode(TB6612_LEFT_MOTOR_BIN1, 'output')
    gpio.setMode(TB6612_LEFT_MOTOR_BIN2, 'output')


def set_left_speed_direction(speed:int, direction:int)->None:
    """Set left speed direction.

    Args:
        speed (int): Speed 0 - 100.
        direction (int): Direction (backwards (-1) or forwards (1))
    """
    assert direction > 0 or direction < 0, f'Direction should be below 0 or above 0, got {direction}'

    # Reverse
    if direction < 0:
        gpio.setDigital(TB6612_LEFT_MOTOR_BIN1, 'OFF')
        gpio.setDigital(TB6612_LEFT_MOTOR_BIN2, 'ON')

    # Forward
    if direction > 0:
        gpio.setDigital(TB6612_LEFT_MOTOR_BIN1, 'ON')
        gpio.setDigital(TB6612_LEFT_MOTOR_BIN2, 'OFF')

    # Set motor spped via PWM signal
    if speed > 100: speed = 100
    if speed < 0: speed = 0
    gpio.setPWM({'pin': TB6612_LEFT_MOTOR_PWMB, 'precentage': speed, 'frequency': 1000})



def set_right_speed_direction(speed:int, direction:int)->None:
    """Set right speed direction

    Args:
        speed (int): Speed 0 - 100.
        direction (int): Direction.
    """
    assert direction > 0 or direction < 0, f'Direction should be below 0 or above 0, got {direction}'

    # Reverse
    if direction < 0:
        gpio.setDigital(TB6612_RIGHT_MOTOR_AIN1, 'OFF')
        gpio.setDigital(TB6612_RIGHT_MOTOR_AIN2, 'ON')

    # Forward
    if direction > 0:
        gpio.setDigital(TB6612_RIGHT_MOTOR_AIN1, 'ON')
        gpio.setDigital(TB6612_RIGHT_MOTOR_AIN2, 'OFF')

    # Set motor spped via PWM signal
    if speed > 100: speed = 100
    if speed < 0: speed = 0
    gpio.setPWM({'pin': TB6612_RIGHT_MOTOR_PWMA, 'precentage': speed, 'frequency': 1000})


def startup_show_led_rainbow()->None:
    everloop = ['black'] * led.length

    led_adjust = 0.0
    if len(everloop) == 35: led_adjust = 0.51   # Matrix creator
    else: led_adjust = 1.01                     # Matrix Voice

    frequency = 0.375
    counter = 0.0
    tick = len(everloop) - 1

    rainbow_start = time()
    rainbow_elapsed = time() - start_time

    while elapsed_time < 30.0:
        # Create rainbow
        for i in range(len(everloop)):
            r = round(max(0, (sin(frequency*counter+(pi/180*240))*155+100)/10))
            g = round(max(0, (sin(frequency*counter+(pi/180*120))*155+100)/10))
            b = round(max(0, (sin(frequency*counter)*155+100)/10))

            counter += led_adjust
            everloop[i] = {'r': r, 'g': g, 'b': b}

            # Slowly show rainbow
            if tick != 0:
                for i in reversed(range(tick)): everloop[i] = {}
                tick -= 1
            
            led.set(everloop)
            sleep(0.35)
        
        rainbow_elapsed = time() - rainbow_start

    sleep(1.0)
    led.set('black')


def main():
    print(f'LEDs on the MATRIX device --> {led.length}')
    startup_show_led_rainbow()

    init_gpio_pins()

    drive_start = time()
    drive_elapsed = time() - drive_start

    while drive_elapsed < 5.0:
        set_right_speed_direction(40, 1)
        set_left_speed_direction(40, 1)

        drive_elapsed = time() - drive_start


if __name__ == "__main__":
    print(__doc__)
    start_time = time()
    main()
    end_time = time() - start_time
    print(f'Done! It took {end_time//60:.0f}m {end_time%60:.1f}s.')
