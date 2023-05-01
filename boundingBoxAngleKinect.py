import cv2
import numpy as np
from collections import defaultdict
import math
import random

### start function definitions###

# find the two largest rectangles and give the length from one end of one to the other


def findLargestCurrRect(rect_dict):
    max_area = -1
    # initializes largest_rect to be the first rect in the keys array of rect_dict
    largest_rect = next(iter(rect_dict))
    for rect in rect_dict.keys():
        dims = rect[1]  # width and height of curr rect
        w = dims[0]
        h = dims[1]
        area = w * h
        if area > max_area:
            max_area = area
            largest_rect = rect

    rect = largest_rect
    box = rect_dict[rect]
    return rect, box

# returns the distance between two points


def distance(x1, y1, x2, y2):
    # Calculate the distance between the two points
    dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    # Return the distance
    return dist

# find the endpoints of the line to be drawn. In theory, these will define the 2D boundary of the rod.


def findEndPointsLine(wood_box, color_box):
    # midpoint of top-left and bottom-left of rectangle
    midpoint_left_wood_box = (wood_box[0]+wood_box[3])//2
    # midpoint of top-right and bottom-right of rectangle
    midpoint_right_wood_box = (wood_box[1]+wood_box[2])//2
    # midpoint of top-left and bottom-left of rectangle
    midpoint_left_color_box = (color_box[0]+color_box[3])//2
    # midpoint of top-right and bottom-right of rectangle
    midpoint_right_color_box = (color_box[1]+color_box[2])//2
    dist_lw_ro = distance(midpoint_left_wood_box[0], midpoint_left_wood_box[1],
                          midpoint_right_color_box[0], midpoint_right_color_box[1])
    dist_lo_rw = distance(midpoint_left_color_box[0], midpoint_left_color_box[1],
                          midpoint_right_wood_box[0], midpoint_right_wood_box[1])
    endpoints = []
    if (dist_lw_ro > dist_lo_rw):  # wood is on the left
        endpoints = [midpoint_left_wood_box, midpoint_right_color_box]
    else:
        endpoints = [midpoint_left_color_box, midpoint_right_wood_box]
    return endpoints


def findEndPointsLineAndCorners(wood_box, color_box):
    # midpoint of top-left and bottom-left of rectangle
    midpoint_left_wood_box = (wood_box[0]+wood_box[3])//2
    # midpoint of top-right and bottom-right of rectangle
    midpoint_right_wood_box = (wood_box[1]+wood_box[2])//2
    # midpoint of top-left and bottom-left of rectangle
    midpoint_left_color_box = (color_box[0]+color_box[3])//2
    # midpoint of top-right and bottom-right of rectangle
    midpoint_right_color_box = (color_box[1]+color_box[2])//2
    dist_lw_ro = distance(midpoint_left_wood_box[0], midpoint_left_wood_box[1],
                          midpoint_right_color_box[0], midpoint_right_color_box[1])
    dist_lo_rw = distance(midpoint_left_color_box[0], midpoint_left_color_box[1],
                          midpoint_right_wood_box[0], midpoint_right_wood_box[1])
    endpoints = []
    corners = []
    if (dist_lw_ro > dist_lo_rw):  # wood is on the left
        endpoints = [midpoint_left_wood_box, midpoint_right_color_box]
        corners = [wood_box[0], wood_box[3], color_box[1], color_box[2]]
    else:
        endpoints = [midpoint_left_color_box, midpoint_right_wood_box]
        corners = [color_box[0], color_box[3], wood_box[1], wood_box[2]]
    return endpoints, corners


def data_for_cylinder_along_z(center_x, center_y, radius, height_z):
    z = np.linspace(0, height_z, 50)
    theta = np.linspace(0, 2*np.pi, 50)
    theta_grid, z_grid = np.meshgrid(theta, z)
    x_grid = radius*np.cos(theta_grid) + center_x
    y_grid = radius*np.sin(theta_grid) + center_y
    return x_grid, y_grid, z_grid


def is_negative():
    rand_int = random.randint(0, 1)
    rand_sign = -1 if rand_int == 0 else 1
    return rand_sign

### end function definitions###
