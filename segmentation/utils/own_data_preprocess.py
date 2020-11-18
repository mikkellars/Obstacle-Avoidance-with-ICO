import cv2
import numpy as np
import os

data_path = "segmentation/data/hallway"
save_dir = 'segmentation/data'

def sort_scene_categories(path:str, save_dir:str)->None:
    def create_folder(folder:str, files:list)->None:
        for f in files:
            im = os.path.join(path, 'images', f)
            im = cv2.imread(im, cv2.IMREAD_COLOR)
            fname = os.path.join(folder, 'images', f)
            cv2.imwrite(fname, im)

            label = os.path.join(path, 'annotations', f)
            label = cv2.imread(label, cv2.IMREAD_GRAYSCALE)
            # label = np.where(label == 1, 4, label)     # Floor
            # label = np.where(label == 2, 1, label)     # Wall
            # label = np.where(label == 3, 42, label)    # Box
            # label = np.where(label == 4, 13, label)    # Person
            fname = os.path.join(folder, 'annotations', f)
            cv2.imwrite(fname, label)

    files = np.array(os.listdir(os.path.join(path, 'images')))
    train_idxs = int(np.floor(0.8 * len(files)))
    val_idxs = len(files) - train_idxs

    idx = np.hstack((np.ones(train_idxs), np.zeros(val_idxs))).astype(np.int32)
    np.random.shuffle(idx)

    train_set = files[idx == 1]
    val_set = files[idx == 0]

    assert set(train_set) != set(val_set), 'Something went wrong, train_set and val_set are equal.'

    train_folder = os.path.join(save_dir, 'train')
    create_folder(train_folder, train_set)

    val_folder = os.path.join(save_dir, 'val')
    create_folder(val_folder, val_set)


sort_scene_categories(data_path, save_dir)