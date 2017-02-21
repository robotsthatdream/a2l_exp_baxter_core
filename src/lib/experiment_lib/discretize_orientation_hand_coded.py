# -*- coding: utf-8 -*-
"""
Created on Thu Jul  7 16:54:57 2016

@author: maestre
"""

import os, sys
run_path = os.path.realpath(os.path.abspath(os.path.join('..', '..')))
sys.path.append(run_path)
import simulation_parameters as sim_param

'''
Given two positions return the orientation of the vector 
between them discretized
'''
def compute_orientation_discr(pos_init_vector, pos_final_vector,
                              current_orien): ## not used
    ## zero means both positions are similar
    same_value_threshold = sim_param.same_orientation_value
                    
        ## same x
    if abs(pos_init_vector[0] - pos_final_vector[0]) < \
       same_value_threshold:        
        if abs(pos_init_vector[1] - pos_final_vector[1]) < \
        same_value_threshold:
            return 'zero'
        elif pos_init_vector[1] < pos_final_vector[1]:
            return 'up'
        elif pos_init_vector[1] > pos_final_vector[1]:
            return 'down'
            
    ## same y
    elif abs(pos_init_vector[1] - pos_final_vector[1]) < \
        same_value_threshold:
        if abs(pos_init_vector[0] - pos_final_vector[0]) < \
        same_value_threshold:
            return 'zero'
        elif pos_init_vector[0] < pos_final_vector[0]:
            return 'right'
        elif pos_init_vector[0] > pos_final_vector[0]:
            return 'left'
            
    ## orient_diff_discrerent x and y (diagonal diff)
    elif pos_init_vector[1] < pos_final_vector[1]:
        if pos_init_vector[0] < pos_final_vector[0]:
            return 'right-up'
        elif pos_init_vector[0] > pos_final_vector[0]:
            return 'left-up'
            
    elif pos_init_vector[1] > pos_final_vector[1]:
        if pos_init_vector[0] < pos_final_vector[0]:
            return 'right-down'
        elif pos_init_vector[0] > pos_final_vector[0]:
            return 'left-down'