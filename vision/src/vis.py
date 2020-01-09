"""
Visualization using visdom.
"""
import numpy as np
import matplotlib.pyplot as plt
from itertools import *

def preview_loader(images, labels, title):
    """
    Shows a few samples of the loader.
    """

    # We generate a plot
    grid_size = int(np.floor(np.sqrt(images.shape[0])))
    (fig, ax) = plt.subplots(11, 11)
    for i in range(11**2):
        current_axis = ax[i//grid_size, i%grid_size]
        current_axis.imshow(images[i, 0])
        current_axis.set_title(labels[i].item())
        current_axis.set_axis_off()
    fig.suptitle(title)
    plt.show()

def preview_kernels(kernels, title):
    """
    Shows a few kernels in 2d
    """

    # We get numpy arrays
    k = kernels.detach().numpy().reshape(-1, 5, 5)

    # We generate a plot
    (fig, ax) = plt.subplots(2, 3)
    for (i, (x, y)) in enumerate(product(range(2), range(3))):
        current_axis = ax[x, y]
        current_axis.imshow(k[i])
        current_axis.set_axis_off()
    fig.suptitle(title)
    plt.show()


def plot_learning_curves(train_loss, test_loss, title):
    """
    Plot learning curves
    """
    fig, ax = plt.subplots() 
    ax.plot(train_loss, color="blue", label="Train-set")
    ax.plot(test_loss, color="green", label="Test-set")
    ax.set_ylabel("Loss")
    ax.set_xlabel("Epochs")
    ax.set_title(title)
    ax.legend()
    plt.show()

def show_image(im, title):
    """
    This function shows an image.
    """
    plt.imshow(im)
    plt.colorbar()
    plt.title(title)
    plt.show()

def show_binary(orig, binarized, title): 
    """
    This function shows the binarized version of the image
    """

    fig, ax = plt.subplots(2) 
    ax[0].imshow(orig)
    ax[0].set_title("Original")
    ax[1].imshow(binarized)
    ax[1].set_title("Binary")
    fig.suptitle(title)
    plt.show()


