# -*- coding: utf-8 -*-
"""
@author: maestre
"""

import os, sys
run_path = os.path.realpath(os.path.abspath(os.path.join('..', '..')))
sys.path.append(run_path)
import simulation_parameters as sim_param

''' 
Compute effect
'''
def compute_effect(current_box_pos):
    effect = ''
    orig_obj_pos = sim_param.obj_pos    
    if round(current_box_pos[0],3) > round(orig_obj_pos[0],3) and \
        orig_obj_pos[1] == current_box_pos[1] and \
        orig_obj_pos[2] == current_box_pos[2]:
            effect = 'right'
#    elif round(orig_obj_pos[0],3) == round(current_box_pos[0] - 0.3,3) and \
    elif round(current_box_pos[0],3) < round(orig_obj_pos[0],3) and \
        orig_obj_pos[1] == current_box_pos[1] and \
        orig_obj_pos[2] == current_box_pos[2]:
            effect = 'left'
#        round(orig_obj_pos[1],3) == round(current_box_pos[1] + 0.3,3) and \
    elif orig_obj_pos[0] == current_box_pos[0] and \
        round(current_box_pos[1],3) > round(orig_obj_pos[1],3) and \
        orig_obj_pos[2] == current_box_pos[2]:
            effect = 'up'
    elif orig_obj_pos[0] == current_box_pos[0] and \
        round(current_box_pos[1],3) < round(orig_obj_pos[1],3) and \
        orig_obj_pos[2] == current_box_pos[2]:
            effect = 'down'
    else:
        print('Effect discretization failed for', orig_obj_pos,
              'regarding the initial pos', current_box_pos)    
    
    return effect
    
'''
Test
'''
if __name__ == '__main__':
    print(compute_effect([0.3, 0.3, 0]))

