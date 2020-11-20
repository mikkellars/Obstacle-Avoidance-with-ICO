"""
Segmentation net interface
"""


import os
import numpy as np
import importlib.util


input_mean = 127.5
input_std = 127.5

colors = np.array([
    [255, 255, 0],
    [0, 0, 255],	# floor - blue
    [0, 255, 0],    # person - green
    [255, 0, 0],    # box	- red
    [0, 0, 0],      # others - black
])


def init_segmentation_net(model_name:str, graph_name:str, use_tpu:bool=False):
    """Init segmentation net

    Args:
        model_name (str): Model name.
        graph_name(str): Graph name.
        use_tpu (bool): Use TPU (default: False).

    Returns:
        (): segmentation network.
    """
    pkg = importlib.util.find_spec('tflite_runtime')
    if pkg:
        from tflite_runtime.interpreter import Interpreter
        if use_tpu:
            from tflite_runtime.interpreter import load_delegate
    else:
        from tensorflow.lite.python.interpreter import Interpreter
        if use_tpu:
            from tensorflow.lite.python.interpreter import load_delegate

    if use_tpu:
        if (graph_name == 'detect.tflite'):
            graph_name = 'edgetpu.tflite'   

    cwd_path = os.getcwd()
    path_to_ckpt = os.path.join(cwd_path, model_name, graph_name)

    if use_tpu: interpreter = Interpreter(model_path=path_to_ckpt, experimental_delegates=[load_delegate('libedgetpu.so.1.0')])
    else: interpreter = Interpreter(model_path=path_to_ckpt)
    interpreter.allocate_tensors()

    return interpreter


def get_labels(model_name:str, labelmap_name:str)->list():
    """Get labels
    Args:
        model_name (str): Model name.
        labelmap_name (str): Label map name.

    Returns:
        (list): labels
    """
    cwd_path = os.getcwd()
    path_to_labels = os.path.join(cwd_path, model_name, labelmap_name)
    with open(path_to_labels, 'r') as f: labels = [l.strip() for l in f.readlines()]
    if labels[0] == '???': del labels[0]
    return labels