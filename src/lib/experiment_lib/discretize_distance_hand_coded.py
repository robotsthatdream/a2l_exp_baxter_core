# -*- coding: utf-8 -*-
"""
Created on Thu Jul  7 16:54:57 2016

@author: maestre
"""
import scipy.spatial.distance as spatial

import os, sys
run_path = os.path.abspath(os.path.join('..', '..'))
sys.path.append(run_path)
import simulation_parameters as sim_param

'''
Compute distance
'''
def compute_distance(eef_pos, current_obj_pos,
                     current_dist): ## not used
    current_dist = spatial.euclidean(eef_pos, 
                                     current_obj_pos[:-1])

    if current_dist <= sim_param.remote_far_boundary_value:
        return 'close'
    elif current_dist <= sim_param.far_close_boundary_value:
        return 'far'
    elif current_dist > sim_param.far_close_boundary_value:
        return 'remote'