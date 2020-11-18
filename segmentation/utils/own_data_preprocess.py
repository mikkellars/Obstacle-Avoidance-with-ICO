import cv2
import numpy as np
import os

data_path = "/home/mikkel/Downloads/test"

def sort_scene_categories(data_path:str):
    files = [f'{data_path}/{f}' for f in os.listdir(data_path)]

    for f in files:
        if not f.endswith('.png'): continue
        
        im = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
        im = np.where(im==1, 4, im) # Floor
        im = np.where(im==2, 42, im)
        im = np.where(im==3, 13, im)
        cv2.imwrite(f, im)
       # cv2.imshow('t',im)
       # cv2.waitKey(0)


sort_scene_categories(data_path)