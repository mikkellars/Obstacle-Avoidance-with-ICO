"""[summary]
Python class to make data_generators for ada20k dataset
Inspiration from https://yann-leguilly.gitlab.io/post/2019-12-14-tensorflow-tfdata-segmentation/
"""

import matplotlib.pyplot as plt
import numpy as np
import tensorflow as tf
import datetime, os
from tensorflow.keras.layers import *
from tensorflow.keras.callbacks import EarlyStopping, ModelCheckpoint, ReduceLROnPlateau
from tensorflow.keras.optimizers import Adam
from tensorflow.keras.applications import MobileNetV2
from tensorflow.keras.metrics import Recall, Precision
from tensorflow.keras.models import Model
import glob
import cv2
from IPython.display import clear_output
import albumentations as A
import random
AUTOTUNE = tf.data.experimental.AUTOTUNE

class gen_ade20k():

    def __init__(self, dataset_path, train_folder, val_folder, batch_size:int = 1, img_size:int = 512, n_channels:int = 3, backbone:str='efficientnetb3'):
        self.img_size = img_size
        self.n_channels = n_channels
        self.min_dataset_classes = self.__get_minimized_ada20k_classes()
        self.n_classes = len(self.min_dataset_classes)
        print(f"The dataset has {self.n_classes} classes")
        self.train_data_size =  len(glob.glob(dataset_path + train_folder + "images/*.png"))
        self.val_data_size =  len(glob.glob(dataset_path + val_folder + "images/*.png"))
        print(f"The Training Dataset contains {self.train_data_size} images.")
        print(f"The Validation Dataset contains {self.val_data_size} images.")
        self.backbone = backbone

       # self.__parse_image("/home/mikkel/Documents/data/ADETrimmed/annotations/training/ADE_train_00006255.png")

        # -- Train Dataset --#
        self.train_dataset = tf.data.Dataset.list_files(dataset_path + train_folder + "images/*.png")
        self.train_dataset = self.train_dataset.map(self.__parse_image)
        self.train_dataset = self.train_dataset.map(self.load_image_train, num_parallel_calls=tf.data.experimental.AUTOTUNE)
        self.train_dataset = self.train_dataset.shuffle(buffer_size=1000)
        self.train_dataset = self.train_dataset.repeat()
        self.train_dataset = self.train_dataset.batch(batch_size)
        self.train_dataset = self.train_dataset.prefetch(buffer_size=AUTOTUNE)

        #-- Validation Dataset --#
        self.val_dataset = tf.data.Dataset.list_files(dataset_path + val_folder + "images/*.png")
        self.val_dataset = self.val_dataset.map(self.__parse_image)
        self.val_dataset = self.val_dataset.map(self.load_image_test)
        self.val_dataset = self.val_dataset.repeat()
        self.val_dataset = self.val_dataset.batch(batch_size)
        self.val_dataset = self.val_dataset.prefetch(buffer_size=AUTOTUNE)


    # -------------------
    # Public functions 
    # -------------------

    def display_sample(self, display_list):
        """Show side-by-side an input image,
        the ground truth and the prediction.
        """
        plt.figure(figsize=(18, 18))

        title = ['Input Image', 'True Mask', 'Predicted Mask']

        for i in range(len(display_list)):
            plt.subplot(1, len(display_list), i+1)
            plt.title(title[i])
            plt.imshow(tf.keras.preprocessing.image.array_to_img(display_list[i]))
            plt.axis('off')
        plt.show()


    def display_sample_opencv(self, img, gr_mask, pr_mask=None):
        """Show side-by-side an input image,
        the ground truth and the prediction.
        Does not work yet
        """
        img_plot = tf.keras.preprocessing.image.array_to_img(img)
        gr_mask_plot = tf.keras.preprocessing.image.array_to_img(gr_mask)

        # Initialize a list of colors to represent each class label
        class_names = []
        for cl in self.min_dataset_classes:
            class_names.append(cl)
        np.random.seed(42)
        colors = np.random.randint(0, 255, size=(self.n_classes, 3), dtype="uint8")
        colors = np.vstack([[0, 0, 0], colors]).astype("uint8") # add black to backround color infront of colors

        # Initialize the legend visualization
        legend = np.zeros(((self.n_classes * 25) + 25, 300, 3), dtype="uint8")
        # Loop over the class names + colors
        for (i, (className, color)) in enumerate(zip(class_names, colors)):
            # draw the class name + color on the legend
            color = [int(c) for c in color]
            cv2.putText(legend, className, (5, (i * 25) + 17),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.rectangle(legend, (100, (i * 25)), (300, (i * 25) + 25),
                tuple(color), -1)

        t = np.array(gr_mask_plot)
        img_plot = cv2.cvtColor(np.array(img_plot), cv2.COLOR_RGB2BGR)
        gr_mask_plot = colors[np.array(gr_mask_plot)]
        gr_mask_plot = cv2.cvtColor(np.array(gr_mask_plot), cv2.COLOR_RGB2BGR)

        cv2.imshow("Legend", legend)
        cv2.imshow("Input", img_plot)
        cv2.imshow("Output", gr_mask_plot)
        cv2.waitKey(0)

        # if pr_mask == None:
        #     title = ['Input Image', 'True Mask', 'Predicted Mask']

        #     for i in range(len(display_list)):
        #         plt.subplot(1, len(display_list), i+1)
        #         plt.title(title[i])
        #         plt.imshow(tf.keras.preprocessing.image.array_to_img(display_list[i]))
        #         plt.axis('off')
        #     plt.show()

        # else:

    def get_datasets(self):
        dataset = {"train": self.train_dataset, "val": self.val_dataset}
        return dataset

    @staticmethod
    def get_indx_to_color():
        # colors = np.array([#[255, 0, 0], # wall - red
        #        [0, 255, 0],	# door - green
        #         [0, 0, 255],	# floor - blue
        #        # [255, 0, 255], # furniture - magenta
        #         [255, 255, 255], # person - yellow
        #         [255, 0, 0], # box	- red
        #         [0, 0, 0], # others - black
        #         ])
        colors = np.array([
            [0, 0, 255],
            [0, 255, 0],
            [255, 0, 0],
            [255, 255, 255],
            [0, 0, 0],
        ])
        return colors
    # -------------------
    # Private functions 
    # -------------------
    def __get_minimized_ada20k_classes(self):
        class_numbers = list(range(0,151))
        min_ada20k_classes = {
            'wall': (2,9,28,33,43,44,145,147, 15, 59),  # <- 9(window), 28(mirror), 33(fence), 43(pillar), 44(sign board), 145(bullertin board), 147(radiator)
            #'door': (15,59), # <- 59(screen door)    
            'floor': (1,7,14,29,30,53,55),     # <- 7(road), 14(ground), 29(rug), 30(field), 53(path), 55(runway)
           # 'furniture': (8,11,16,19,20,24,25,31,34,36,45,46), # <- 8(bed), 11(cabinet), 14(sofa), 16(table), 19(curtain), 20(chair), 25(shelf), 31(armchair), 34(desk), 36(wardrobe), 45(dresser), 46(counter) 
            'person' : (4),
            'box': (3) 
        }

        # Adds remaining classes to dict
        for cl in min_ada20k_classes:
            val = min_ada20k_classes[cl]
            if not isinstance(val,int):
                for num in val:
                    if num in class_numbers:
                        class_numbers.remove(num)
            else:
                if val in class_numbers:
                    class_numbers.remove(val)
       # 38 75 150 150
        min_ada20k_classes['others'] = tuple(class_numbers)
        
        return min_ada20k_classes

    
    def __minimize_mask(self, mask):
        """Minimizes the classes present in the mask given specified classes.
        Also makes "not label" which has a value equal to 255, to have a value of 0

        Args:
            mask ([type]): [description]
        """
        # In scene parsing, "not labeled" = 255, but it will mess up with our N_CLASS = 150, since 255 means the 255th class Which doesn't exist
        
        new_mask = mask
        #new_mask = tf.where(mask == 255, np.dtype('uint8').type(0), new_mask)

        for idx, cl in enumerate(self.min_dataset_classes):
            val = self.min_dataset_classes[cl]
            master_class = idx
            if not isinstance(val,int):
                #master_class = val[0] 
                for num in val:
                    new_mask = tf.where(mask == num, np.dtype('uint8').type(master_class), new_mask) 
            else:
                new_mask = tf.where(mask == val, np.dtype('uint8').type(master_class), new_mask) 
        return new_mask


    def __channel_mask(self, mask):
        """Converts from 1 channel with a label, to each number of channels representing a class

        Args:
            mask ([type]): [description]
        """
        class_labels = list(range(0, self.n_classes))
        one_hot_map = list()

        for cl in class_labels:
            cl_map = tf.reduce_all(tf.equal(mask, cl), axis=-1)
            one_hot_map.append(cl_map)
            
        one_hot_map = tf.stack(one_hot_map, axis=-1)
        one_hot_map = tf.cast(one_hot_map, tf.float32)

        return one_hot_map

    def __parse_image(self, img_path: str) -> dict:
        """Load an image and its annotation (mask) and returning
        a dictionary.

        Parameters
        ----------
        img_path : str
            Image (not the mask) location.

        Returns
        -------
        dict
            Dictionary mapping an image and its annotation.
        """
        image = tf.io.read_file(img_path)
        image = tf.image.decode_jpeg(image, channels=3)
        image = tf.image.convert_image_dtype(image, tf.uint8)

        # For one Image path:
        # .../trainset/images/training/ADE_train_00000001.jpg
        # Its corresponding annotation path is:
        # .../trainset/annotations/training/ADE_train_00000001.png
        mask_path = tf.strings.regex_replace(img_path, "images", "annotations")
        mask_path = tf.strings.regex_replace(mask_path, "jpg", "png")
        mask = tf.io.read_file(mask_path)
        # The masks contain a class index for each pixels
        mask = tf.image.decode_png(mask, channels=1)
        mask = tf.image.convert_image_dtype(mask, tf.uint8)
        mask_new = self.__minimize_mask(mask)
        mask_new = self.__channel_mask(mask_new) # Make each label to each own channel
        return {'image': image, 'segmentation_mask': mask_new}

    @tf.function
    def normalize(self, input_image: tf.Tensor, input_mask: tf.Tensor) -> tuple:
        """Rescale the pixel values of the images between 0.0 and 1.0
        compared to [0,255] originally.

        Parameters
        ----------
        input_image : tf.Tensor
            Tensorflow tensor containing an image of size [SIZE,SIZE,3].
        input_mask : tf.Tensor
            Tensorflow tensor containing an annotation of size [SIZE,SIZE,1].

        Returns
        -------
        tuple
            Normalized image and its annotation.
        """
        input_image = tf.cast(input_image, tf.float32) / 255.0
        return input_image, input_mask

    @tf.function
    def load_image_train(self, datapoint:dict) -> tuple:
        """Apply some transformations to an input dictionary
        containing a train image and its annotation.

        Notes
        -----
        An annotation is a regular  channel image.
        If a transformation such as rotation is applied to the image,
        the same transformation has to be applied on the annotation also.

        Parameters
        ----------
        datapoint : dict
            A dict containing an image and its annotation.

        Returns
        -------
        tuple
            A modified image and its annotation.
        """
        input_image = datapoint['image']
        input_mask = datapoint['segmentation_mask']

        if tf.random.uniform(()) > 0.5:
            input_image = tf.image.flip_left_right(input_image)
            input_mask = tf.image.flip_left_right(input_mask)

        if tf.random.uniform(()) > 0.5:
            input_image = tf.image.random_crop(input_image, size=[self.img_size//4, self.img_size//4, 3], seed=1)
            input_mask = tf.image.random_crop(input_mask, size=[self.img_size//4, self.img_size//4, self.n_classes], seed=1)

        # Only image augmentations
        if tf.random.uniform(()) > 0.5: # Saturation
            input_image = tf.image.random_saturation(input_image, 0.1, 3.0)

        if tf.random.uniform(()) > 0.5: # Brightness
            input_image = tf.image.random_brightness(input_image, 0.1, 1.0)

        if tf.random.uniform(()) > 0.5: # Contrast
            input_image = tf.image.random_contrast(input_image, 0.1, 0.5)

        if tf.random.uniform(()) > 0.5: # HUE
            input_image = tf.image.random_hue(input_image, 0.2)        

        input_image, input_mask = self.normalize(input_image, input_mask)

        input_image = tf.image.resize(input_image, (self.img_size, self.img_size))
        input_mask = tf.image.resize(input_mask, (self.img_size, self.img_size))

        return input_image, input_mask


    @tf.function
    def load_image_test(self, datapoint: dict) -> tuple:
        """Normalize and resize a test image and its annotation.

        Notes
        -----
        Since this is for the test set, we don't need to apply
        any data augmentation technique.

        Parameters
        ----------
        datapoint : dict
            A dict containing an image and its annotation.

        Returns
        -------
        tuple
            A modified image and its annotation.
        """
        input_image = tf.image.resize(datapoint['image'], (self.img_size, self.img_size))
        input_mask = tf.image.resize(datapoint['segmentation_mask'], (self.img_size, self.img_size))

        input_image, input_mask = self.normalize(input_image, input_mask)

        return input_image, input_mask

        
# DATA_PATH = '/home/mikkel/Documents/data/ADETrimmed'
# datagenerator = gen_ade20k(DATA_PATH, '/images/training/', '/images/validation/')
# for image, mask in dataset['val'].take(1):
#     sample_image, sample_mask = image, mask

# datagenerator.display_sample((sample_image[0], sample_mask[0]))

