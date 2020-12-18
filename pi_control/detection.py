"""
"""


import time
import cv2
import copy
import numpy as np
import matplotlib.pyplot as plt


class Detector:

    def detect_floor(
        self,
        inp:np.array,
        ksize:int=3,
    )->dict():
        ret = dict()

        x = copy.deepcopy(inp)

        kernel = np.ones((ksize, ksize), np.uint8)
        x = cv2.erode(x, kernel)
        x = cv2.dilate(x, kernel)

        max_area = -1
        cnts, _ = cv2.findContours(x.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in cnts:
            area = cv2.contourArea(cnt)
            if area < max_area: continue

            ret['area'] = area

            moments = cv2.moments(cnt)
            ret['cx'] = int(moments['m10'] / moments['m00'])
            ret['cy'] = int(moments['m01'] / moments['m00'])
            
            ret['bbox'] = cv2.boundingRect(cnt)

        return ret

    def extract_data(
        self,
        inp:np.array,
        ksize:int=3,
        erode_iter:int=1,
        dilate_iter:int=1,
        cnt_min_area:int=100
    )->list:
        ret = list()
        im = copy.deepcopy(inp)

        kernel = np.ones((ksize, ksize), np.uint8)
        im = cv2.erode(im, kernel, iterations=erode_iter)
        im = cv2.dilate(im, kernel, iterations=dilate_iter)

        cnts, _ = cv2.findContours(im.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in cnts:
            area = cv2.contourArea(cnt)
            if area < cnt_min_area: continue

            d = dict()
            d['area'] = area
            moments = cv2.moments(cnt)
            d['cx'] = int(moments['m10'] / moments['m00'])
            d['cy'] = int(moments['m01'] / moments['m00'])
            d['bbox'] = cv2.boundingRect(cnt)
            ret.append(d)
        
        return ret

    def find_closest_obstacle(
        self, 
        data:list
    )->bool:
        max_y = -1
        for d in data:
            (x, y, w, h) = d['bbox']
            if y + h > max_y:
                max_y = y + h
        return max_y

    def detect_collisions(
        self,
        mask:np.array, 
        collision_thresh:int=90,
        show:bool=False
    )->tuple:
        if show:
            show_im = copy.deepcopy(mask)
        
        oh, ow = mask.shape
        floor = np.where(mask == 1, 255, 0).astype(np.uint8)
        box = np.where(mask == 3, 255, 0).astype(np.uint8)
        wall = np.where(mask == 0, 255, 0).astype(np.uint8)

        # floor_data = self.detect_floor(floor)
        # if show:
        #     (x, y, w, h) = floor_data['bbox']
        #     cx, cy = floor_data['cx'], floor_data['cy']
        #     cv2.rectangle(show_im, (x,y), (x+w,y+h), (255,255,255), 1)
        #     cv2.circle(show_im, (cx, cy), 1, (255,255,255), -1)

        left, right = 0, 0

        box_data = self.extract_data(box, cnt_min_area=100)
        for d in box_data:
            (x, y, w, h) = d['bbox']
            cx, cy = d['cx'], d['cy']

            if show:
                cv2.rectangle(show_im, (x,y), (x+w,y+h), (255,255,255), 1)
                cv2.circle(show_im, (cx, cy), 1, (255,255,255), -1)
                cv2.circle(show_im, (cx, y+h), 1, (255,255,255), -1)

            if cx < ow//2 and y+h > left: left = y+h
            elif cx > ow//2 and y+h > right: right = y+h
            else: right = y+h
        
        wall_data = self.extract_data(wall, cnt_min_area=250)
        for d in wall_data:
            (x, y, w, h) = d['bbox']
            cx, cy = d['cx'], d['cy']

            if show:
                cv2.rectangle(show_im, (x,y), (x+w,y+h), (255,255,255), 1)
                cv2.circle(show_im, (cx, cy), 1, (255,255,255), -1)
                cv2.circle(show_im, (cx, y+h), 1, (255,255,255), -1)

            if cx < ow//2 and y+h > left: left = y+h
            elif cx > ow//2 and y+h > right: right = y+h
            else: right = y+h     

        if show:
            plt.imshow(show_im, cmap='gray'), plt.axis('off'), plt.show()

        collision = 0
        if left > right and left > collision_thresh:
            collision = -1
        elif left < right and right > collision_thresh:
            collision = 1

        left /= oh
        right /= oh

        return collision, left, right

    def detect_human(
        self,
        mask:np.array, 
        reflect_thresh:int=80)->tuple:

        oh, ow = mask.shape
        human = np.where(mask == 2, 255, 0).astype(np.uint8)
        human_data = self.extract_data(human, cnt_min_area=250)
        left = 0
        right = 0

        for d in human_data:
            (x, y, w, h) = d['bbox']
            cx, cy = d['cx'], d['cy']

            if cx < ow//2 and ow - cx > left:
                left = ow - cx
            elif cx > ow//2 and cx > right:
                right = cx

        reflect = 0
        if left > right and left > reflect_thresh:
            reflect = -1
        elif left < right and right > reflect_thresh:
            reflect = 1

        left /= ow
        right /= ow

        return reflect, left, right

    def detect_human_bbox(
        self,
        bboxes,
        scores,
        reflect_thresh:int=0.8)->tuple:

        #TODO make dynamic
        ow, oh = 320, 320

        left, right = 0, 0

        for box in bboxes: # box[0] (ymin), box[1] (xmin), box[2] (ymax), box[3] (xmax) (normalized)
            bbox_width = box[3] - box[1] #(box[3] * ow) - (box[1] * ow)
            cx = box[1] + (bbox_width/2) #(box[1] * ow) + (bbox_width / 2) # center of box
            bbox_height = box[2] - box[0] #(box[2] * oh) - (box[0] * oh)
            cy = box[0] + (bbox_height/2) #(box[0] * oh) + (bbox_height / 2) # center of box
            if cx < 0.5: #ow//2: # left side of image
                # print(f'Left: cy {cy:.3f} cx {1.0 - cx:.3f}')
                # if cy > left:
                #     left = cy
                if 1.0 - cx > left:
                    left = 1.0 - cx
            elif cx > 0.5: #ow//2: # right side of image
                # print(f'Right: cy {cy:.3f} cx {cx:.3f}')
                # if cy > right:
                #     right = cy
                if cx > right:
                    right = cx
            else: # middle of image
               # print(f'Middle: cy {cy:.3f} cx {cx:.3f}')
                # if cy > right:
                #     right = cy
                if cy > left:
                    left = cy

        reflect = 0
        if left > right and left > reflect_thresh:
            reflect = -1
        elif left < right and right > reflect_thresh:
            reflect = 1

        print(f'Left {left:.3f} Right {right:.3f} Reflect {reflect}')

        return reflect, left, right

if __name__ == "__main__":
    print(__doc__)
    
    import os
    import matplotlib.pyplot as plt

    start_time = time.time()

    masks = [
        f'/home/pi/project_in_ai/segmentation/test/images/masks/{f}'
        for f in os.listdir('/home/pi/project_in_ai/segmentation/test/images/masks')
    ]

    detector = Detector()

    for mask in masks:
        mask = cv2.imread(mask, cv2.IMREAD_COLOR)
        mask = cv2.cvtColor(mask, cv2.COLOR_BGR2RGB)

        collision, left, right = detector.detect_collisions(mask, True)

        print(f'Collision: {collision}')
        print(f'Left:\n{left}')
        print(f'Right:\n{right}')

    end_time = time.time() - start_time
    print(f'Done! It took {end_time//60:.0f} minuttes and {end_time%60:.2f} seconds.')
