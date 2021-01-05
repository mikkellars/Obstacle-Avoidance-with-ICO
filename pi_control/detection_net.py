""" Scripts for detecting humans on TPU
"""

"""
Segmentation net interface
"""


import os
import cv2
import numpy as np
import importlib.util


input_mean = 127.5
input_std = 127.5

class detection_net:
    def __init__(self, model_name:str, graph_name:str, use_tpu:bool=True):
        self.model_name = model_name
        self.graph_name = graph_name
        self.use_tpu = use_tpu

        self.interpreter = self.__init_detection_net()
        # Init model details
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        self.height = self.input_details[0]['shape'][1]
        self.width = self.input_details[0]['shape'][2]
        self.floating_model = (self.input_details[0]['dtype'] == np.float32)

    def __init_detection_net(self):
        """Init segmentation net

        Args:
            model_name (str): Model name.
            graph_name(str): Graph name.
            use_tpu (bool): Use TPU (default: False).

        Returns:
            interpreter(): segmentation network.
        """
        pkg = importlib.util.find_spec('tflite_runtime')
        if pkg:
            from tflite_runtime.interpreter import Interpreter
            if self.use_tpu:
                from tflite_runtime.interpreter import load_delegate
        else:
            from tensorflow.lite.python.interpreter import Interpreter
            if self.use_tpu:
                from tensorflow.lite.python.interpreter import load_delegate

        if self.use_tpu:
            if (self.graph_name == 'detect.tflite'):
                self.graph_name = 'edgetpu.tflite'   

        cwd_path = os.getcwd()
        path_to_ckpt = os.path.join(cwd_path, self.model_name, self.graph_name)

        if self.use_tpu: interpreter = Interpreter(model_path=path_to_ckpt, experimental_delegates=[load_delegate('libedgetpu.so.1.0')])
        else: interpreter = Interpreter(model_path=path_to_ckpt)
        interpreter.allocate_tensors()

        return interpreter

    def __prepare_img(self, img):
        input_mean = 127.5
        input_std = 127.5

        frame = img.copy()
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_resized = cv2.resize(frame_rgb, (self.width, self.height))
        input_data = np.expand_dims(frame_resized, axis=0)

        # Normalize pixel values if using a floating model (i.e. if model is non-quantized)
        if self.floating_model:
            input_data = (np.float32(input_data) - input_mean) / input_std

        return input_data

    def run_inference(self, img):   
        input_data = self.__prepare_img(img)
        # Perform the actual detection by running the model with the image as input
        self.interpreter.set_tensor(self.input_details[0]['index'],input_data)
        self.interpreter.invoke()

        
        classes = self.interpreter.get_tensor(self.output_details[1]['index'])[0] # Class index of detected objects
        # Retrieve detection results for only person class
        idx = np.where(classes == 0)
        
        boxes = self.interpreter.get_tensor(self.output_details[0]['index'])[0][idx] # Bounding box coordinates of detected objects
        scores = self.interpreter.get_tensor(self.output_details[2]['index'])[0][idx] # Confidence of detected objects
        
        # Extracting prediction over some score
        scores_idx = np.where(scores > 0.5)
        scores = scores[scores_idx]
        boxes = boxes[scores_idx]
        #num = interpreter.get_tensor(output_details[3]['index'])[0]  # Total number of detected objects (inaccurate and not needed)
        return boxes, scores
        
    @staticmethod
    def show_results(image, boxes, scores):
        # Loop over all detections and draw detection box if confidence is above minimum threshold
        imH, imW, _ = image.shape 
        for i in range(len(scores)):
            if ((scores[i] > 0.5) and (scores[i] <= 1.0)):

                # Get bounding box coordinates and draw box
                # Interpreter can return coordinates that are outside of image dimensions, need to force them to be within image using max() and min()
                ymin = int(max(1,(boxes[i][0] * imH)))
                xmin = int(max(1,(boxes[i][1] * imW)))
                ymax = int(min(imH,(boxes[i][2] * imH)))
                xmax = int(min(imW,(boxes[i][3] * imW)))
                
                cv2.rectangle(image, (xmin,ymin), (xmax,ymax), (10, 255, 0), 2)

                # Draw label
                object_name = 'Person' # Look up object name from "labels" array using class index
                label = '%s: %d%%' % (object_name, int(scores[i]*100)) # Example: 'person: 72%'
                labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2) # Get font size
                label_ymin = max(ymin, labelSize[1] + 10) # Make sure not to draw label too close to top of window
                cv2.rectangle(image, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), (255, 255, 255), cv2.FILLED) # Draw white box to put label text in
                cv2.putText(image, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2) # Draw label text

        # # All the results have been drawn on the image, now display the image
        # image = cv2.resize(image, (256, 256))
        # cv2.imshow('Object detector', cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
