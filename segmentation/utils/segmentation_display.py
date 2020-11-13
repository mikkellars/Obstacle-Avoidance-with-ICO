import tensorflow as tf
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

def display_sample(display_list, colors=None):
    """Show side-by-side an input image,
    the ground truth and the prediction.
    """
    plt.figure(figsize=(18, 18))

    title = ['Input Image', 'True Mask', 'Predicted Mask']

    for i in range(len(display_list)):
        plt.subplot(1, len(display_list), i+1)
        plt.title(title[i])
        img = tf.keras.preprocessing.image.array_to_img(display_list[i])

        # Coverts to colors if mask 
        if(title[i] != 'Input Image' and colors.all() != None):
            np_img = display_list[i].numpy()
            np_img = np.float32(colors[np_img])
            np_img = np.squeeze(np_img, axis=2)
            img = Image.fromarray(np_img.astype('uint8'), 'RGB')

        plt.imshow(img)
        plt.axis('off')
    plt.show()


def create_mask(pred_mask: tf.Tensor) -> tf.Tensor:
    """Return a filter mask with the top 1 predicitons
    only.

    Parameters
    ----------
    pred_mask : tf.Tensor
        A [IMG_SIZE, IMG_SIZE, N_CLASS] tensor. For each pixel we have
        N_CLASS values (vector) which represents the probability of the pixel
        being these classes. Example: A pixel with the vector [0.0, 0.0, 1.0]
        has been predicted class 2 with a probability of 100%.

    Returns
    -------
    tf.Tensor
        A [IMG_SIZE, IMG_SIZE, 1] mask with top 1 predictions
        for each pixels.
    """
    pred_mask = tf.argmax(tf.squeeze(pred_mask), axis=-1)
    pred_mask = tf.expand_dims(pred_mask, axis=-1)

    return pred_mask
    

def show_predictions(model, dataset=None ,num=1, colors=None):
    """Show a sample prediction.

    Parameters
    ----------
    dataset : [type], optional
        [Input dataset, by default None
    num : int, optional
        Number of sample to show, by default 1
    """
    if dataset:
        for image, mask in dataset.take(num):
            pred_mask = model.predict(image)
            display_sample([image[0], create_mask(mask), create_mask(pred_mask)], colors)