"""Training Unet model and converting to tf.lite
"""

import os
import sys
sys.path.append(os.getcwd())

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
from PIL import Image
AUTOTUNE = tf.data.experimental.AUTOTUNE

import segmentation_models as sm
from segmentation_models.utils import set_trainable

from segmentation.utils.data_generator import gen_ade20k
from segmentation.utils.segmentation_display import *

def parse_arguments():
    import argparse
    parser = argparse.ArgumentParser(description='')
    parser.add_argument('--model_name', type=str, default='unet', help='name of trained model')
    parser.add_argument('--backbone', type=str, default='resnet50')
    parser.add_argument('--data_root', type=str, default= '/home/mikkel/Documents/data/ADETrimmed', help='path to dataset')
    parser.add_argument('--save_dir_model', type=str, default='segmentation/unet/models', help='path to save models')
    parser.add_argument('--save_dir_logs', type=str, default='segmentation/unet/logs', help='logs path for tensorboard')
   #"" parser.add_argument('--epochs', type=int, default=10, help='number of epochs (default: 100)')
    args = parser.parse_args()
    return args


def create_represent_data(data):
    def data_gen():
        for i in data:
            yield [list([i])]
    return data_gen
            

def save_quantified_model(model, dataset):
    # quantization to easier run on pi or edgetpu
    data = list()
    for image, _ in dataset['train'].take(10):
        data.append(image[0])
    data = np.array(data)
    
    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    converter.optimizations = [tf.lite.Optimize.DEFAULT]
    converter.representative_dataset = create_represent_data(data)
    # Ensure that if any ops can't be quantized, the converter throws an error
    converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
    # Set the input and output tensors to uint8 (APIs added in r2.3)
    # converter.inference_input_type = tf.uint8
    # converter.inference_output_type = tf.uint8
    converter.allow_custom_ops = True
    converter.experimental_new_converter = False

    tflite_model_quant = converter.convert()

    open(f'{args.save_dir_model}/best_model_unet_quant.tflite' , "wb").write(tflite_model_quant)


def save_tf_lite_model(model):
    # OPS! CAN ONLY CONVERT THE MODEL WITH TENSORFLOW 2.1
    converter = tf.lite.TFLiteConverter.from_keras_model(model) 
    converter.optimizations = [tf.lite.Optimize.DEFAULT]
    tfmodel = converter.convert()
    open(f'{args.save_dir_model}/best_model_unet.tflite' , "wb").write(tfmodel)

def main(args):
        
    # Data download
    img_size = 96
    n_channels = 3
    batch_size = 1

    DATA_PATH = args.data_root
    datagenerator = gen_ade20k(DATA_PATH, '/images/training/', '/images/validation/',batch_size, img_size, n_channels)
    dataset = datagenerator.get_datasets()

    N_CLASSES = datagenerator.n_classes

    # Callbacks for training
    callbacks = [
        # to collect some useful metrics and visualize them in tensorboard
        # tensorboard_callback,
        # if no accuracy improvements we can stop the training directly
        #tf.keras.callbacks.EarlyStopping(patience=10, verbose=1),
        # to save checkpoints
        tf.keras.callbacks.ModelCheckpoint(f'{args.save_dir_model}/best_model_{args.model_name}.h5', verbose=1, save_best_only=True, save_weights_only=True)
    ]

    # define model
    BACKBONE = args.backbone
    EPOCHS = 100
    STEPS_PER_EPOCH = datagenerator.train_data_size // batch_size
    VALIDATION_STEPS = datagenerator.val_data_size // batch_size

    preprocess_input = sm.get_preprocessing(BACKBONE)

    model = sm.PSPNet(BACKBONE, encoder_weights='imagenet',  encoder_freeze=True, classes=N_CLASSES, input_shape=(img_size, img_size, n_channels))
    # Segmentation models losses can be combined together by '+' and scaled by integer or float factor
    # set class weights for dice_loss (car: 1.; pedestrian: 2.; background: 0.5;)
    dice_loss = sm.losses.DiceLoss(class_weights=np.array([1, 1, 0.5])) 
    focal_loss = sm.losses.BinaryFocalLoss() if N_CLASSES == 1 else sm.losses.CategoricalFocalLoss()
    total_loss = dice_loss + (1 * focal_loss)

    # actulally total_loss can be imported directly from library, above example just show you how to manipulate with losses
    # total_loss = sm.losses.binary_focal_dice_loss # or sm.losses.categorical_focal_dice_loss 

    metrics = [sm.metrics.IOUScore(threshold=0.5), sm.metrics.FScore(threshold=0.5)]

    # compile keras model with defined optimozer, loss and metrics
    model.compile(
        'Adam',
        #loss=total_loss,
        loss=sm.losses.cce_jaccard_loss,
        metrics=metrics,
    )
    
    #pretrain model decoder (frozen encoder)
    model.fit(dataset['train'],
              epochs=100,
              steps_per_epoch=STEPS_PER_EPOCH,
              validation_steps=VALIDATION_STEPS,
              validation_data=dataset['val'],
              callbacks=callbacks)

    # # release all layers for training
    # set_trainable(model) # set all layers trainable and recompile model

    # continue training
   
   # model.load_weights(f'{args.save_dir_model}/best_model_unet.h5') # Getting best weights based on saved validation model
    # model.fit(dataset['train'], epochs=100,
    #                     steps_per_epoch=STEPS_PER_EPOCH,
    #                     validation_steps=VALIDATION_STEPS,
    #                     validation_data=dataset['val'],   
    #                     callbacks=callbacks)

    model.load_weights(f'{args.save_dir_model}/best_model_unet.h5') # Getting best weights based on saved validation model
    save_tf_lite_model(model)
    #save_quantified_model(model, dataset) # Does not support keras.softmax beacuse it uses tf.reduce_max
    show_predictions(model, dataset['val'], datagenerator.val_data_size, datagenerator.get_indx_to_color())

if __name__ == '__main__':
    print(__doc__)
    args = parse_arguments()
    main(args)
