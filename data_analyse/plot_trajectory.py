"""Plot trajectory of Trashy.
"""

import os
import sys
sys.path.append(os.getcwd())

import time
import cv2
import numpy as np
import matplotlib.pyplot as plt


class BBoxWidget:
    def __init__(self, image):
        self.original_image = image
        self.clone = self.original_image.copy()

        cv2.namedWindow('image')
        cv2.setMouseCallback('image', self.extract_coordinates)

        # Bounding box reference points
        self.image_coordinates = []

    def extract_coordinates(self, event, x, y, flags, parameters):
        # Record starting (x,y) coordinates on left mouse button click
        if event == cv2.EVENT_LBUTTONDOWN:
            self.image_coordinates = [(x,y)]

        # Record ending (x,y) coordintes on left mouse button release
        elif event == cv2.EVENT_LBUTTONUP:
            self.image_coordinates.append((x,y))
            print('top left: {}, bottom right: {}'.format(self.image_coordinates[0], self.image_coordinates[1]))
            print('x,y,w,h : ({}, {}, {}, {})'.format(self.image_coordinates[0][0], self.image_coordinates[0][1], self.image_coordinates[1][0] - self.image_coordinates[0][0], self.image_coordinates[1][1] - self.image_coordinates[0][1]))

            # Draw rectangle 
            cv2.rectangle(self.clone, self.image_coordinates[0], self.image_coordinates[1], (36,255,12), 2)
            cv2.imshow("image", self.clone) 

        # Clear drawing boxes on right mouse button click
        elif event == cv2.EVENT_RBUTTONDOWN:
            self.clone = self.original_image.copy()

    def show_image(self): return self.clone
    
    def get_bbox(self): return self.image_coordinates


class GenerateMask:
    max_value = 255
    max_value_H = 360//2
    low_H = 0
    low_S = 0
    low_V = 0
    high_H = max_value_H
    high_S = max_value
    high_V = max_value
    window_capture_name = 'Video Capture'
    window_detection_name = 'Object Detection'
    low_H_name = 'Low H'
    low_S_name = 'Low S'
    low_V_name = 'Low V'
    high_H_name = 'High H'
    high_S_name = 'High S'
    high_V_name = 'High V'

    def __init__(self, frame):
        self.frame = frame

    def on_low_H_thresh_trackbar(self, val):
        self.low_H = val
        self.low_H = min(self.high_H - 1, self.low_H)
        cv2.setTrackbarPos(self.low_H_name, self.window_detection_name, self.low_H)

    def on_high_H_thresh_trackbar(self, val):
        self.high_H = val
        self.high_H = max(self.high_H, self.low_H + 1)
        cv2.setTrackbarPos(self.high_H_name, self.window_detection_name, self.high_H)

    def on_low_S_thresh_trackbar(self, val):
        self.low_S = val
        self.low_S = min(self.high_S - 1, self.low_S)
        cv2.setTrackbarPos(self.low_S_name, self.window_detection_name, self.low_S)

    def on_high_S_thresh_trackbar(self, val):
        self.high_S = val
        self.high_S = max(self.high_S, self.low_S + 1)
        cv2.setTrackbarPos(self.high_S_name, self.window_detection_name, self.high_S)

    def on_low_V_thresh_trackbar(self, val):
        self.low_V = val
        self.low_V = min(self.high_V - 1, self.low_V)
        cv2.setTrackbarPos(self.low_V_name, self.window_detection_name, self.low_V)

    def on_high_V_thresh_trackbar(self, val):
        self.high_V = val
        self.high_V = max(self.high_V, self.low_V + 1)
        cv2.setTrackbarPos(self.high_V_name, self.window_detection_name, self.high_V)

    def create(self):
        cv2.namedWindow(self.window_capture_name)
        cv2.namedWindow(self.window_detection_name)
        cv2.createTrackbar(self.low_H_name, self.window_detection_name, self.low_H, self.max_value_H, self.on_low_H_thresh_trackbar)
        cv2.createTrackbar(self.high_H_name, self.window_detection_name, self.high_H, self.max_value_H, self.on_high_H_thresh_trackbar)
        cv2.createTrackbar(self.low_S_name, self.window_detection_name, self.low_S, self.max_value, self.on_low_S_thresh_trackbar)
        cv2.createTrackbar(self.high_S_name, self.window_detection_name , self.high_S, self.max_value, self.on_high_S_thresh_trackbar)
        cv2.createTrackbar(self.low_V_name, self.window_detection_name , self.low_V, self.max_value, self.on_low_V_thresh_trackbar)
        cv2.createTrackbar(self.high_V_name, self.window_detection_name , self.high_V, self.max_value, self.on_high_V_thresh_trackbar)

        while True:
            frame_HSV = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
            frame_threshold = cv2.inRange(frame_HSV, (self.low_H, self.low_S, self.low_V), (self.high_H, self.high_S, self.high_V))
            
            cv2.imshow(self.window_capture_name, self.frame)
            cv2.imshow(self.window_detection_name, frame_threshold)
            
            key = cv2.waitKey(30)
            if key == ord('q') or key == 27:
                break
        
        return frame_threshold


def get_trajectory(file:str)->tuple:
    cap = cv2.VideoCapture(file)

    ret, frame = cap.read()
    frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_CUBIC)

    bbox_widget = BBoxWidget(image=frame)
    while True:
        cv2.imshow('image', bbox_widget.show_image())
        key = cv2.waitKey(1)
        if key == ord('q'):
            break
    
    bbox = bbox_widget.get_bbox()
    x, y = bbox[0][0], bbox[0][1]
    w, h = bbox[1][0] - x, bbox[1][1] - y
    track_window = (x, y, w, h)

    cv2.destroyAllWindows()
    
    roi = frame[y:y+h, x:x+w]
    hsv_roi =  cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    generate_mask = GenerateMask(frame=roi)
    mask = generate_mask.create()

    roi_hist = cv2.calcHist([hsv_roi], [0], mask, [180], [0, 180])
    cv2.normalize(roi_hist, roi_hist, 0, 255, cv2.NORM_MINMAX)

    cv2.destroyAllWindows()
    
    xs, ys = list(), list()
    term_crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1 )
    while(1):
        ret, frame = cap.read()
        if ret == True:
            im = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_CUBIC)
            hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
            dst = cv2.calcBackProject([hsv], [0], roi_hist, [0,180], 1)
            
            ret, track_window = cv2.meanShift(dst, track_window, term_crit)
            
            x,y,w,h = track_window
            im = cv2.rectangle(im, (x,y), (x+w,y+h), 255, 2)
            cv2.imshow('Image', im)

            xs.append(x + w/2)
            ys.append(y + h/2)
            
            k = cv2.waitKey(30)
            if k == ord('q'):
                break
        else:
            break

    cv2.destroyAllWindows()

    return np.array(xs), np.array(ys)


if __name__ == '__main__':
    print(__doc__)
    start_time = time.time()

    save_dir = '/home/mathias/Documents/project_in_ai/data_analyse/assets/test1'

    files = [
        '/home/mathias/Documents/project_in_ai/data_analyse/data/test1/01.mp4',
        '/home/mathias/Documents/project_in_ai/data_analyse/data/test1/02.mp4',
        '/home/mathias/Documents/project_in_ai/data_analyse/data/test1/03.mp4',
    ]

    for i, f in enumerate(files):
        xs, ys = get_trajectory(f)
        plt.plot(xs, ys, label=str(i))
    
    plt.grid()
    plt.legend()
    plt.xticks([])
    plt.yticks([])
    plt.savefig(f'{save_dir}/trajectory.png')
    plt.show()
    plt.close('all')

    end_time = time.time() - start_time
    print(f'It took {end_time//60} minutes and {end_time%60.0} seconds')
    exit(0)