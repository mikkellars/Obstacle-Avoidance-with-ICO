"""
Controller for the robot.
"""


import os
import cv2
import numpy as np
from time import sleep, time
from math import pi, sin
from matrix_lite import gpio
from matrix_lite import led
from camera import VideoStream

# from picamera import PiCamera
# from picamera.array import PiRGBArray


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

res_w, res_h = 640, 480


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


def startup_show_led_rainbow()->None:
    everloop = ['black'] * led.length

    led_adjust = 0.0
    if len(everloop) == 35: led_adjust = 0.51   # Matrix creator
    else: led_adjust = 1.01                     # Matrix Voice

    frequency = 0.375
    counter = 0.0
    tick = len(everloop) - 1

    rainbow_start = time()
    rainbow_elapsed = time() - rainbow_start

    while rainbow_elapsed < 5.0:
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
            sleep(0.1)
        
        rainbow_elapsed = time() - rainbow_start

    sleep(1.0)
    led.set('black')


def cur_img_num(path):
    cur_img_num = len(os.listdir(path))
    cur_img_num + 1
    return cur_img_num


def main():
    print(f'LEDs on the MATRIX device --> {led.length}')
    startup_show_led_rainbow()

    init_gpio_pins()

    frame_rate_calc = 1
    freq = cv2.getTickFrequency()
    video_stream = VideoStream(resolution=(res_w, res_h),framerate=30).start()
    sleep(1)

    path ='pi_control/samples/python/controller/images/'
    img_num = cur_img_num(path)
    # Super loop
    i = 0
    while True:
        start_tick = cv2.getTickCount()

        # ----------------
        # Get camera input
        # ----------------

        frame = video_stream.read()
        frame = cv2.flip(frame, -1)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        #frame = cv2.resize(frame, (256,256))
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        cv2.imshow('Real image', frame)
        # Stop if key 'q' pressed
        key = cv2.waitKey(30)
        if key == ord('q'):
            break
        if key == ord('n'):
            cv2.imwrite(f'{path}/image_{img_num:03d}.png', frame)
            img_num += 1


    # -----------
    # Stop robot
    # -----------

    cv2.destroyAllWindows()
    video_stream.stop()


if __name__ == "__main__":
    print(__doc__)
    start_time = time()
    main()
    end_time = time() - start_time
    print(f'Done! It took {end_time//60:.0f}m {end_time%60:.1f}s.')
