#! /usr/bin/env python

from solid2.extensions.bosl2 import *
from solid2.extensions.bosl2 import metric_screws
from subprocess import run
import numpy as np


# basic stuff
def bolt():
    return metric_screws.metric_bolt(size=20, headtype="hex", l=40)


# extrude test function
def extrude_along_path():
    # path = [ [0, 0, 0], [100, 100, 10], [200, 100, 0], [300, 300, 0], [400,400,0] ]
    path = arc(n=20, d=600, angle=90)
    ushape = [[-10, 0], [-10, 10], [-7, 10], [-7, 2], [7, 2], [7, 7], [10, 7], [10, 0]]
    trackshape = [
        [0, 0],
        [20, 0],
        [20, 10],
        [15, 10],
        [15, 5],
        [10, 5],
        [10, 10],
        [0, 10],
        [-10, 10],
        [-10, 5],
        [-15, 5],
        [-15, 10],
        [-20, 10],
        [-20, 0],
    ]

    return path_sweep(trackshape, path)


# extrude function with parameters
def extrude_along_path_new(diameter, deg, cross_section):
    """
    Input:
        Diameter: curve diameter
        deg: curve degree
        cross_section: a array of points which represent the cross section of the track

    """
    # path = [ [0, 0, 0], [100, 100, 10], [200, 100, 0], [300, 300, 0], [400,400,0] ]
    path = arc(n=20, d=diameter, angle=deg)
    ushape = [[-10, 0], [-10, 10], [-7, 10], [-7, 2], [7, 2], [7, 7], [10, 7], [10, 0]]
    trackshape = [
        [0, 0],
        [20, 0],
        [20, 10],
        [15, 10],
        [15, 5],
        [10, 5],
        [10, 10],
        [0, 10],
        [-10, 10],
        [-10, 5],
        [-15, 5],
        [-15, 10],
        [-20, 10],
        [-20, 0],
    ]

    return path_sweep(cross_section, path)


# A python function to generate curved toy train track
def gen_curve_track(trackshape, diameter, angle):
    """
    Input:
        Diameter: curve diameter
        angle: curve degree
        trackshape: a array of points which represent the cross section of the track

    This function will generate a complete toy train track 3d model with parameter setting.
    """

    rad = (90 - angle) / 180 * np.pi
    if angle >= 0:
        return union()(
            extrude_along_path_new(diameter, angle, trackshape)
            + cylinder(r=5.5, h=10).right(diameter / 2).fwd(15)
            + cube([5, 20, 10]).right(diameter / 2 - 2.5).fwd(10)
            - cylinder(r=6.5, h=11)
            .right(diameter / 2 * np.sin(rad) + 13 * np.cos(rad))
            .back(diameter / 2 * np.cos(rad) - 13 * np.sin(rad))
            .down(0.5)
            - cube([7, 15, 30], anchor=CENTER)
            .rotate(angle)
            .right((diameter / 2) * np.sin(rad))
            .back((diameter / 2) * np.cos(rad))
        )
    else:
        return union()(
            extrude_along_path_new(diameter, angle, trackshape)
            + cylinder(r=5.5, h=10).right(diameter / 2).back(15)
            + cube([7, 20, 10]).right(diameter / 2 - 2.5).fwd(10)
            - cylinder(r=6.5, h=11)
            .right(diameter / 2 * np.sin(rad) - 13 * np.cos(rad))
            .back(diameter / 2 * np.cos(rad) + 13 * np.sin(rad))
            .down(0.5)
            - cube([7, 15, 30], anchor=CENTER)
            .rotate(angle)
            .right((diameter / 2) * np.sin(rad))
            .back((diameter / 2) * np.cos(rad))
        )


# test code to generate a track

trackshape = [
    [0, 0],
    [20, 0],
    [20, 10],
    [15, 10],
    [15, 7],
    [10, 7],
    [10, 10],
    [0, 10],
    [-10, 10],
    [-10, 7],
    [-15, 7],
    [-15, 10],
    [-20, 10],
    [-20, 0],
]
diameter = 900
angle = 20

rad = (90 - angle) / 180 * np.pi

# test code for shape union
# assembly = union()(extrude_along_path_new(diameter,angle,trackshape)+sphere(r=7).right(diameter/2).fwd(12).up(7) +\
#             cube([5,30,8]).right(diameter/2-2.5).fwd(13).up(1)-\
#                 cylinder(r=6.5, h=11).right(diameter/2*np.sin(rad)+12*np.cos(rad)).back(diameter/2*np.cos(rad)-12*np.sin(rad)).down(0.5)-\
#                     cube([7,15,30],anchor=CENTER).rotate(angle).right((diameter/2)*np.sin(rad)).back((diameter/2)*np.cos(rad)))

assembly = gen_curve_track(trackshape, diameter, angle)


assembly.save_as_scad(filename="generator_test.scad")

# a command line to call scad to convert 3d model to stl file
run(["openscad", "-o", "generator_test1.stl", "generator_test.scad"])
