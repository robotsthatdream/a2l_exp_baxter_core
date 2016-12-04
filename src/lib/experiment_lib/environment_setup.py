# -*- coding: utf-8 -*-
"""
@author: maestre
"""

from __future__ import print_function

import random
from math import sin,cos,pi
import numpy as np
import scipy.spatial.distance as spatial

import os, sys
run_path = os.path.abspath(os.path.join('..', '..'))
sys.path.append(run_path)
import simulation_parameters as sim_param

'''
Generate the initial positions of the eef
'''
def gen_init_eef(nb_init_pos):
    nb_init_pos += 2
    list_radians = [0]
    tmp = np.linspace(pi/(180.0/360), pi/180, nb_init_pos-1)
    tmp2 = tmp[:-1].tolist()
    tmp2.reverse()
    list_radians = list_radians + tmp2

#    print(list_radians)
        
    list_x_axis = []
    list_y_axis = []    
    for a in list_radians[1:]:
        list_x_axis.append(cos(a))
        list_y_axis.append(sin(a))    
#    list_z_axis = [sim_param.eef_z_value for i in range(len(list_x_axis))]
    list_z_axis = np.zeros(len(list_x_axis))
    return list_x_axis, list_y_axis, list_z_axis

'''
Generate random pos of the object
'''
def gen_obj_pos():
    obj_pos = [round(random.uniform(-0.5,0.5),2),
               round(random.uniform(-0.5,0.5),2),
                0]    

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
#    print (gen_init_eef(18))
    print (compute_nb_wps([0.0, 0.0], [1.0, 1.0]))