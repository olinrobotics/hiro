#!/usr/bin/env python
"""
These functions are borrowed from ROS tf transform
as PointCloud2 transformation is not supported for now
"""
import numpy as np

import tf.transformations


def transformPointCloud(coords, angle, height):
    """
    Transform point cloud
    :param coords: numpy array of coords
    :param angle: angle of the camera
    :param height: height of the camera
    :return: transformed coords
    """
    mat44 = asMatrix(angle, height)

    def xf(p):
        x, y, z = p
        if not np.math.isnan(x):
            xyz = np.asarray(np.dot(mat44, np.array([x, y, z, 1.0]))[:3])
        else:
            xyz = np.asarray([x, y, z])
        return xyz

    r = [xf(p) for p in coords]
    return r


def asMatrix(angle, height):
    """

    :param angle: angle of the camera
    :param height: height of the camera
    :return: transformation matrix
    """
    translation = [0, -height, 0]
    rotation = tf.transformations.quaternion_from_euler(-np.radians(90 - angle), 0, 0)
    return fromTranslationRotation(translation, rotation)


def fromTranslationRotation(translation, rotation):
    """

    :param translation: translation expressed as a tuple (x,y,z)
    :param rotation: rotation quaternion expressed as a tuple (x,y,z,w)
    :return: a :class:`numpy.matrix` 4x4 representation of the transform

    Converts a transformation from :class:`tf.Transformer` into a representation as a 4x4 matrix.
    """

    return np.dot(tf.transformations.translation_matrix(translation),
                  tf.transformations.quaternion_matrix(rotation))
