"""
Trashy, your friendly neighbourhood trashcan.
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
from ico_node import ICO
from datalogger import DataLogger
from segmentation_net import (
    init_segmentation_net,
    get_labels,
    colors
)
from detection_net import detection_net

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
    sleep(2)

    # Create segmentation net
    model_name = 'pi_control/Sample_TFLite_model'
    graph_name = 'best_model_pspnet.tflite' #'best_model_pspnet_quant.tflite'
    interpreter = init_segmentation_net(model_name, graph_name, use_tpu=False)
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    height = input_details[0]['shape'][1]
    width = input_details[0]['shape'][2]
    floating_model = (input_details[0]['dtype'] == np.float32)

    # Create detection net
    model_name = 'pi_control/Sample_TFLite_model'
    graph_name = 'ssdlite_mobiledet_coco_edgetpu.tflite' #'best_model_pspnet_quant.tflite'
    det_net = detection_net(model_name, graph_name, use_tpu=True)

    # # Get labels
    # labelmap_name = 'labelmap.txt'
    # labels = get_labels(model_name, labelmap_name)

    # Create detector
    detector = Detector()

    # Init ICO's
    left_obs_ico = ICO(lr=0.1)
    right_obs_ico = ICO(lr=0.1)

    left_human_ico = ICO(lr=0.1)
    right_human_ico = ICO(lr=0.1)

    # Init data loggers
    log_collision = DataLogger('pi_control/logs/obs_col.txt')
    log_left_right = DataLogger('pi_control/logs/left_right.txt')
    log_human = DataLogger('pi_control/logs/human.txt')
    log_human_col = DataLogger('pi_control/logs/human_col.txt')

    # ----------
    # Super loop
    # ----------

    i = 0
    while True:
        if i % 100 == 0: start_tick = cv2.getTickCount()

        # Get camera input
        frame = video_stream.read()
        frame = cv2.flip(frame, -1)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Running detection net
        boxes, scores = det_net.run_inference(frame)
        det_net.show_results(frame, boxes, scores)

        # Segment input
        input_data = cv2.resize(frame, (width, height))
        input_data = np.expand_dims(input_data, axis=0)

        if floating_model: input_data = (np.float32(input_data)) / 255.0

        interpreter.set_tensor(input_details[0]['index'],input_data)
        interpreter.invoke()
        masks = interpreter.get_tensor(output_details[0]['index'])[0]
        masks = np.array(masks)
        masks = masks.argmax(axis=2)

        # Detect obstacle
        collision, left, right = detector.detect_collisions(masks, show=False, collision_thresh=80)

        left_obs_val = left_obs_ico.run_and_learn(1 if collision == -1 else 0, left)
        right_obs_val = right_obs_ico.run_and_learn(1 if collision == 1 else 0, right)

        # Save input, output, and weight
        log_collision.write_data(i, [1 if collision == -1 else 0, 1 if collision == 1 else 0])
        log_left_right.write_data(i, [left, right, left_obs_ico.weight_predic, right_obs_ico.weight_predic, left_obs_val, right_obs_val,])

        # Detect human
        # reflect, left, right = detector.detect_human(masks, reflect_thresh=60) # ON THE SEGMENTATION
        reflect, left, right = detector.detect_human_bbox(boxes, scores, reflect_thresh=0.8) # ON THE DETECTION
        left_human_val = left_human_ico.run_and_learn(1 if reflect == -1 else 0, left)
        right_human_val = right_human_ico.run_and_learn(1 if reflect == 1 else 0, right)
        
        # Save input, output, and weight
        log_human.write_data(i, [left, right, left_human_ico.weight_predic, right_human_ico.weight_predic, left_human_val, right_human_val,])
        log_human_col.write_data(i, [1 if reflect == -1 else 0, 1 if reflect == 1 else 0])

        # # Running both obstacle avoidance and human following
        # left_mc_val = 50 * (left_obs_val + right_human_val)
        # right_mc_val = 50 * (right_obs_val + left_human_val)

        # Running only human following
        if len(scores) > 0:
            left_mc_val = 100 * (right_human_val)
            right_mc_val = 100 * (left_human_val)
        else:
            left_mc_val = 50
            right_mc_val = 50
        


        # # TEST!
        # left_mc_val = 0
        # right_mc_val = 0

        if left_mc_val > 100: left_mc_val = 100
        if right_mc_val > 100: right_mc_val = 100

        # Set motor values
        if collision != 0:
            # Reflex evade obstacle
            set_right_speed_direction(0, 1)
            set_left_speed_direction(0, 1)
            # sleep(0.25)
            if collision == -1: # Left
                # Right around obstable
                set_left_speed_direction(30, 1)
                #sleep(0.5)
            elif collision == 1: # Right
                # Left around obstable
                set_right_speed_direction(30, 1)
                #sleep(0.5)
        else:
            set_right_speed_direction(right_mc_val, 1)
            set_left_speed_direction(left_mc_val, 1)

        # Show image
        masks = np.float32(colors[masks])
        masks = cv2.resize(masks, (256,256))
        masks = cv2.cvtColor(masks, cv2.COLOR_RGB2BGR)

        frame = cv2.resize(frame, (256,256))
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR, cv2.INTER_NEAREST)
        frame = cv2.addWeighted(frame.astype(np.uint8), 1.0, masks.astype(np.uint8), 0.5, 0.0)
        cv2.imshow('Real image', frame)
      #  cv2.imshow('Mask image', masks)

        # Stop if key 'q' pressed
        key = cv2.waitKey(1)
        if key == ord('q'):
            break
        elif key == ord('s'):
            set_right_speed_direction(0, 1)
            set_left_speed_direction(0, 1)
            cv2.waitKey()

        if i % 100 == 0:
            # Calculate framerate
            end_tick = cv2.getTickCount()
            frame_rate_calc = 1 / ((end_tick - start_tick) / freq)
            print(f'Frame rate: {frame_rate_calc:.3f}')

        # Update i
        i += 1
        

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
