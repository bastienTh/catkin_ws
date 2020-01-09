"""
This module contains utilities functions used in the preprocessing phase of the detection.
"""
import imageio
import glob
import matplotlib.pyplot as plt
import numpy as np
import skimage.transform as tf
from skimage.filters import threshold_otsu
from skimage.measure import (approximate_polygon, find_contours)
from skimage.morphology import (closing, convex_hull_object, square)
from skimage.segmentation import clear_border
from scipy.spatial import ConvexHull

try:
    import vis
except ImportError:
    import vis


def rescale(imag, debug=False):
    """
    This function returns a rescaled version of the image.
    This means that values all lies in [0, 1].
    """

    imag = imag.copy()
    imag = imag/255.0
    # We rescale the pixels to have minimum value of 0 and max value of 1.
    

    if debug:
        plt.imshow(imag, cmap="gray")
        plt.title("rescaled image")
        plt.colorbar()
        plt.show()

    return imag

def binarize(imag, debug=False):
    """
    This function returns a binarized version of the image.
    """

    imag = imag.copy()

    # We compute an optimal threshold and form a binary image
    th_value = threshold_otsu(imag)
    binar = imag > th_value
    binar = closing(binar, square(20))
    clear_border(binar)
    binar = convex_hull_object(binar)
    
    if debug:
        fig, ax = plt.subplots() 
        #ax.imshow(imag, cmap='gray')
        ax.imshow(binar, alpha=0.4, cmap='gray')
        fig.suptitle("Binary image")
        plt.show()

    return binar


def approximate_square(contour):
    """
    This function approximates a contour with a square.
    """

    tol = 50

    # While the right number of segments is not found, we modify the tolerance consequently.
    for _ in range(50):
        coords = approximate_polygon(contour, tolerance=tol)
        coords = np.flip(coords[0:-1], axis=1)
        if coords.shape[0] == 4:
            return coords
        if coords.shape[0] < 4:
            print("Failed to approximate square with tolerance {}, found {} points. Retrying."\
                .format(tol, coords.shape[0]))
            tol -= 1
        else:
            print("Failed to approximate square with tolerance {}, found {} points. Retrying."\
                .format(tol, coords.shape[0]))
            tol += 1

    raise Exception("Failed to approximate square")


def reorder_contour(contour):
    """
    This function allows to reorder the contour so that the down-right point is always first, and
    points are ordered clockwise.
    """

    # We reorder the points


    contour_x = np.array(contour)[:,0]
    contour_y = np.array(contour)[:,1]

    sum = contour_x + contour_y
    index_1 = np.where(sum == np.amax(sum))[0][0]
    index_3 = np.where(sum == np.amin(sum))[0][0]
    min_x=1000.0
    min_index_x=0
    max_x = 0.0
    max_index_x=0
    for i in range(4):
        if (not(i==index_1 or i==index_3)):
            if (contour_x[i]<min_x):
                min_x = contour_x[i]
                min_index_x = i
            if (contour_x[i]>max_x):
                max_x = contour_x[i]
                max_index_x = i
    index_2 = min_index_x
    index_4 = max_index_x
    contour = np.array([
        contour[index_1].tolist(),
        contour[index_2].tolist(),
        contour[index_3].tolist(),
        contour[index_4].tolist()
    ])


    return contour


def get_box_contours(imag, debug=False):
    """
    This function takes as input a single channel image, and returns a list of contours bounding
    the cubes.
    """

    # We make sure that we work on a local copy of the image
    imag = imag.copy()

    # We turn the image to a binary one
    binar = binarize(imag)

    # We extract the contours, and keep only the largest ones.
    ctrs = find_contours(binar, 0.5)
    surface =[]
    for ctr in ctrs:
        surface.append(ConvexHull(ctr).area)
    mean = sum(surface)/len(surface)
    
    ctrs = [ctr for i, ctr in enumerate(ctrs) if surface[i] > mean/2]


    # We approximate the contours by squares and reorder the points
    # Use `approximate_square`
    ctrs = [approximate_square(c) for c in ctrs]
    ctrs = [reorder_contour(c) for c in ctrs]


    if debug:
        plt.imshow(imag)
        plt.imshow(binar, alpha=0.4)
        for coords in ctrs:
            plt.plot(coords[:,0], coords[:, 1], 'og', linewidth=2)
            plt.plot(coords.mean(axis=0)[0], coords.mean(axis=0)[1], 'or')
            ind = [1, 2, 3, 4]
            for i, txt in enumerate(ind):
                plt.annotate(txt, (coords[i, 0], coords[i, 1]))
        plt.title("Contours found")
        plt.show()

    return ctrs


def get_sprites(imag, ctrs, debug=False):
    """
    This function computes a projective transform from the source (mnist image) to
    the destination (contour) and extracts the warped sprite.
    """

    # We make sure that we work on a local copy of the image
    imag = imag.copy()

    # We loop through the sprites 
    sprts = []

    for contour in ctrs:

        # We compute the projective transform

        destination_points = np.array(
            [
                [28, 28],
                [0, 28],
                [0, 0],
                [28, 0]
            ]
        )

        tform = tf.estimate_transform('similarity', contour, destination_points)


        # We transform the image

        warped = tf.warp(imag, inverse_map=tform.inverse)[:28, :28]

        if debug:
            _, axis = plt.subplots(nrows=2, figsize=(8, 3))
            axis[0].imshow(imag)
            axis[0].plot(destination_points[:, 0], destination_points[:, 1], '.r')
            axis[1].imshow(warped)
            plt.show()

        sprts.append(warped)

    return sprts


def preprocess_sprites(sprts, debug=False):
    """
    This function preprocesses sprites to make them closer to the mnist images.
    """

    out_sprites = []

    for imag in sprts:

        # We make a local copy
        imag = imag.copy()

        # We rescale, inverse and normalize.

        imag = 1.0 - imag
        imag = imag - imag.mean()
        imag = imag/imag.std()

        if debug:
            plt.imshow(imag)
            plt.title("Pre-processed sprites")
            plt.colorbar()
            plt.show()

        out_sprites.append(imag)

    return out_sprites


if __name__ == "__main__":

    print("0) Getting images:") 
    test_data = glob.glob('data/cubes/*/*.jpg')
    print("Found test images: {}".format(test_data))
    images = []
    for path in test_data:
        images.append(imageio.imread(path)[:, :, 0])
    images = np.array(images)
    vis.show_image(images[0], "Image sample")

    print("Ok\n\n1) Rescaling image")
    rescaled = rescale(images[0], debug=False)
    assert rescaled.max() <= 1.
    assert rescaled.min() >= 0. 

    #print("OK\n\n2) Binarizing image")
    #for im in images:
    #    binarize(im, debug=False)

    print("OK\n\n3) Getting boxes")
    ctrs = []
    for im in images:
        ctrs.append(get_box_contours(im, debug=False))

    print("OK\n\n4) Getting sprites")
    sprites = []
    for i in range(images.shape[0]):
        sprites.append(get_sprites(images[i], ctrs[i], debug=False))

    print("OK\n\n5) Pre-processing")
    for sprt in sprites:
        preprocess_sprites(sprt, debug=True)

