#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
This script loads an image, extracts sprites, and infer the classes with the network trained earlier.
"""
from src.models import LeNet
from src.detection import get_box_contours, get_sprites, preprocess_sprites
import imageio
import torch
import collections
import matplotlib.pyplot as plt
import rospy

import numpy as np

from ros4pro.srv import VisionInfer, VisionInferResponse

LABELS=[1, 2]

Box = collections.namedtuple('Box', 'contour sprite label')

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    """
    angle between two vectors 
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

def detect_box_and_number(image):

    labels = [1, 2]

    checkpoint_path = "checkpoints/final-15:24:57.t7"

    model = LeNet(classes=labels)
    model.load_state_dict(torch.load(checkpoint_path))

    boxes = process(image, model)

    res = VisionInferResponse()
    
    for box in boxes:
        res.label = box.label

        center = box.contour.mean(axis=0)

        res.x.append(center[0])
        res.y.append(center[1])
        
        length = np.linalg.norm(box.contour[0], box.contour[1])

        ref_point = np.array([length/2 + center[0], length/2 + center[1]])

        res.theta.append(angle_between(ref_point, box.contour[0]))


    return res

def detect_box_and_number_server():
    rospy.init_node('detect_box_and_number_server')
    s = rospy.Service('detect_box_and_number_server', VisionInfer, detect_box_and_number)


def process(image, model, debug=None):
    """
    This function processes an image given a model, and returns a list of Box.
    """

    debug_inner = debug in ["inner", "all"]
    contours = get_box_contours(image, debug=debug_inner)
    sprites = get_sprites(image, contours, debug=debug_inner)
    inputs = preprocess_sprites(sprites, debug=debug_inner)
    labels = [model.infer(i) for i in inputs]

    boxes = [Box(contour=c, sprite=s, label=l) for c, s, l in  zip(contours, sprites, labels)]

    if debug in ["all", "synthesis"]:
        for box in boxes:
            fig, ax = plt.subplots(nrows=2)
            ax[0].imshow(image)
            ax[0].plot(box.contour[:,0], box.contour[:,1], "og")
            ax[0].plot(box.contour.mean(axis=0)[0], box.contour.mean(axis=0)[1], "og")
            ax[1].imshow(box.sprite)
            ax[1].set_title("Label recognized: {}".format(box.label))
            plt.show()

    return boxes




if __name__ == "__main__":

    detect_box_and_number_server()

    # import glob

    # labels = [1, 2]

    # checkpoint_path = "checkpoints/final-15:24:57.t7"

    # model = LeNet(classes=labels)
    # model.load_state_dict(torch.load(checkpoint_path))
    # model.eval()

    # test = glob.glob('src/data/cubes/*/*.jpg')

    # for path in test:
    #     print("Testing image {}".format(path))
    #     image = imageio.imread(path)[:,:,0]
    #     try:
    #         boxes = process(image, model, debug="synthesis")
    #     except Exception:
    #         print("Failed to process image {}".format(path))
    #         pass
