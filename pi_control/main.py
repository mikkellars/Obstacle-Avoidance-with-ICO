"""
Controller for Trashy.
"""


# -------
# IMPORTS
# -------

# Global
import os
import cv2
import numpy as np
from time import sleep, time

# Local
from camera import VideoStream
from detection import Detector
from segmentation_net import (
    init_segmentation_net,
    get_labels,
    colors
)
from matrix_gpio import (
    init_gpio_pins,
    set_left_speed_direction,
    set_right_speed_direction,
    startup_show_led_rainbow
)


# ---------
# VARIABLES
# ---------

min_conf_thresh = 0.5
res_w, res_h = 1280, 720


# ----
# MAIN
# ----

def main():

    # -----
    # Setup
    # -----

    # On startup show rainbow
    startup_show_led_rainbow()
    
    # Init GPIO pins
    init_gpio_pins()

    # Create video stream
    frame_rate_calc = 1
    freq = cv2.getTickFrequency()
    video_stream = VideoStream(resolution=(res_w, res_h),framerate=30).start()
    sleep(1)

    # Create segmentation net
    model_name = 'pi_control/samples/python/controller/Sample_TFLite_model'
    graph_name = 'best_model_pspnet.tflite'
    interpreter = init_segmentation_net(model_name, graph_name)
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    height = input_details[0]['shape'][1]
    width = input_details[0]['shape'][2]
    floating_model = (input_details[0]['dtype'] == np.float32)

    # # Get labels
    # labelmap_name = 'labelmap.txt'
    # labels = get_labels(model_name, labelmap_name)

    # Create detector
    detector = Detector()

    # ----------
    # Super loop
    # ----------

    i = 0
    while True:
        start_tick = cv2.getTickCount()

        # Get camera input
        frame = video_stream.read()
        frame = cv2.flip(frame, -1)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Segment input
        input_data = cv2.resize(frame, (width, height))
        input_data = np.expand_dims(input_data, axis=0)

        if floating_model: input_data = (np.float32(input_data)) / 255.0

        interpreter.set_tensor(input_details[0]['index'],input_data)
        interpreter.invoke()
        masks = interpreter.get_tensor(output_details[0]['index'])[0]
        masks = np.array(masks)
        masks = masks.argmax(axis=2)
        masks = np.float32(colors[masks])

        # Detect obstacle
        collision, left, right = detector.detect_collisions(masks, False)

        # Set motor values
        # if collision != 0:
        #     # Relex evade obstacle
        #     set_right_speed_direction(0, 1)
        #     set_left_speed_direction(0, 1)
        #     sleep(0.1)
        #     time = 1.5
        #     if collision == -1: # Left
        #         # Right around obstable
        #         set_left_speed_direction(30, 1)
        #         sleep(time)
        #         # Forward
        #         set_right_speed_direction(30, 1)
        #         set_left_speed_direction(30, 1)
        #         sleep(time)
        #         # Left turn onto path
        #         set_right_speed_direction(30, 1)
        #         set_left_speed_direction(15, 1)
        #         sleep(time)
        #     elif collision == 1: # Right
        #         # Left around obstable
        #         set_right_speed_direction(30, 1)
        #         sleep(time)
        #         # Forward
        #         set_right_speed_direction(30, 1)
        #         set_left_speed_direction(30, 1)
        #         sleep(time)
        #         # Right turn onto path
        #         set_right_speed_direction(15, 1)
        #         set_left_speed_direction(30, 1)
        #         sleep(time)
        # else:
        #     set_right_speed_direction(30, 1)
        #     set_left_speed_direction(30, 1)
        masks = cv2.resize(masks, (256,256))
        masks = cv2.cvtColor(masks, cv2.COLOR_RGB2BGR)
        # cv2.imshow('Object detector', masks)

        frame = cv2.resize(frame, (256,256))
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        frame = cv2.addWeighted(frame.astype(np.uint8), 1.0, masks.astype(np.uint8), 0.5, 0.0)
        cv2.imshow('Real image', frame)
        # Stop if key 'q' pressed
        key = cv2.waitKey(30)
        if key == ord('q'):
            break

        # Calculate framerate
        end_tick = cv2.getTickCount()
        frame_rate_calc = 1 / ((end_tick - start_tick) / freq)
        print(f'Frame rate: {frame_rate_calc:.3f}')
        

    # -----------
    # Stop robot
    # -----------

    set_right_speed_direction(0, 1)
    set_left_speed_direction(0, 1)
    cv2.destroyAllWindows()
    video_stream.stop()


if __name__ == "__main__":
    print(__doc__)
    start_time = time()
    main()
    end_time = time() - start_time
    print(f'Done! It took {end_time//60:.0f}m {end_time%60:.1f}s.')
