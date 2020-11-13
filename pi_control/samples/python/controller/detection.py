"""
"""


import time
import cv2
import copy
import numpy as np


def extract_data(inp:np.array, ksize:int=5, erode_iter:int=2, dilate_iter:int=2, cnt_min_area:int=50)->list:
    ret = list()
    im = copy.deepcopy(inp)

    kernel = np.ones((ksize, ksize), np.uint8)
    inp = cv2.erode(inp, kernel, iterations=erode_iter)
    inp = cv2.dilate(inp, kernel, iterations=dilate_iter)

    cnts, _ = cv2.findContours(im, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in cnts:
        moments = cv2.moments(cnt)
        cx = int(moments['m10'] / moments['m00'])
        cy = int(moments['m01'] / moments['m00'])
        area = cv2.contourArea(cnt)
        x, y, w, h = cv2.boundingRect(cnt)
        
        if area < cnt_min_area: continue
        
        ret.append({'cx': cx, 'cy': cy, 'area': area, 'bbox': (x, y, w, h)})
    
    return ret


def check_for_collision(data:list, collision_thresh:int=250)->bool:
    for d in data:
        (x, y, w, h) = d['bbox']
        if y+h > collision_thresh:
            return True
    return False


def detect_collisions(im:np.array, show:bool=False):
        if show:
            show_im = copy.deepcopy(mask)
        
        # Remove yellow color and other stuff
        oh, ow, oc = mask.shape
        for y in range(0, oh):
            for x in range(0, ow):
                pixel = mask[y, x]
                if pixel[0] != 255 and pixel[1] != 0 and pixel[2] != 0:
                    mask[y, x] = (0, 0, 0)
                elif pixel[0] != 0 and pixel[1] != 0 and pixel[2] != 255:
                    mask[y, x] = (0, 0, 0)

        floor = mask[:, :, 2]
        box = mask[:, :, 0]

        floor_data = extract_data(floor, cnt_min_area=1000)
        for d in floor_data:
            (x, y, w, h) = d['bbox']
            cx, cy = d['cx'], d['cy']

            if show:
                cv2.rectangle(show_im, (x,y), (x+w,y+h), (0,255,0), 1)
                cv2.circle(show_im, (cx, cy), 1, (0,255,0), -1)


        left, right = list(), list()
        box_data = extract_data(box, dilate_iter=5)
        for d in box_data:
            (x, y, w, h) = d['bbox']
            cx, cy = d['cx'], d['cy']

            if show:
                cv2.rectangle(show_im, (x,y), (x+w,y+h), (0,255,0), 1)
                cv2.circle(show_im, (cx, cy), 1, (0,255,0), -1)
                cv2.circle(show_im, (cx, y+h), 1, (0,255,0), -1)

            if cx < ow//2:
                left.append(d)
            elif cx > ow//2:
                right.append(d)
            else:
                left.append(d)
                right.append(d)

        if show:
            plt.imshow(show_im), plt.axis('off'), plt.show()

        collision = 0
        if check_for_collision(left):
            collision = -1
        elif check_for_collision(right):
            collision = 1

        return collision, left, right


if __name__ == "__main__":
    print(__doc__)
    
    import os
    import matplotlib.pyplot as plt

    start_time = time.time()

    masks = [f'segmentation/test/images/masks/{f}' for f in os.listdir('segmentation/test/images/masks')]

    for mask in masks:
        mask = cv2.imread(mask, cv2.IMREAD_COLOR)
        mask = cv2.cvtColor(mask, cv2.COLOR_BGR2RGB)

        collision, left, right = detect_collisions(mask, False)

        print(f'Collision: {collision}')
        print(f'Left:\n{left}')
        print(f'Right:\n{right}')

    end_time = time.time() - start_time
    print(f'Done! It took {end_time//60:.0f} minuttes and {end_time%60:.2f} seconds.')
