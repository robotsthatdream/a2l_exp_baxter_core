# -*- coding: utf-8 -*-
"""
@author: maestre
"""

from __future__ import print_function

import random
from math import sin,cos,pi,degrees
import numpy as np
import scipy.spatial.distance as spatial

import os, sys
run_path = os.path.realpath(os.path.abspath(os.path.join('..', '..')))
sys.path.append(run_path)
import simulation_parameters as sim_param

'''
Generate the initial positions of the eef
'''
def gen_init_eef(nb_init_pos, 
                 radius, 
                 obj_pos):
                     
    if sim_param.single_init_pos:
        return [sim_param.untucked_left_eef_pos[0]], \
               [sim_param.untucked_left_eef_pos[1]], \
               [sim_param.untucked_left_eef_pos[2]]

    else:                     
        angle = 2*pi/nb_init_pos    
        list_radians = [angle*i for i in range(0,nb_init_pos)]
            
        list_x_axis = []
        list_y_axis = []    
        for a in list_radians:
            list_x_axis.append(obj_pos[0] + cos(a)*radius)
            list_y_axis.append(obj_pos[1] + sin(a)*radius)    
#        list_z_axis = [obj_pos[2] for i in range(len(list_x_axis))]
        list_z_axis = [sim_param.eef_z_value for i in range(len(list_x_axis))]              
        
        return list_x_axis, list_y_axis, list_z_axis

'''
Generate random pos of the object
'''
def gen_obj_pos():
    obj_pos = [round(random.uniform(-0.5,0.5),2),
               round(random.uniform(-0.5,0.5),2),
                -0.11]
    return obj_pos

'''
Compute number of wps given 2 positions
'''
def compute_nb_wps(point_a, point_b):
   dist = spatial.euclidean(point_a, point_b)
   return dist // sim_param.step_length
    
    
    
'''
Test
'''
if __name__== "__main__":
#    print (gen_init_eef(4))
    print (gen_init_eef(4, 0.2, [0.65, 0.1, 0.14]))
#    print (compute_nb_wps([0.0, 0.0], [1.0, 1.0]))