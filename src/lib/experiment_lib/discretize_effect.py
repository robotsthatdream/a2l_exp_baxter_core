# -*- coding: utf-8 -*-
"""
@author: maestre
"""

from __future__ import print_function

import os, sys
run_path = os.path.realpath(os.path.abspath(os.path.join('..', '..')))
sys.path.append(run_path)
import simulation_parameters as sim_param

#''' 
#Compute effect
#'''
#def compute_effect(orig_obj_pos,
#                   current_box_pos):
#    effect = ''
#    if round(current_box_pos[0],3) > round(orig_obj_pos[0],3) and \
#        orig_obj_pos[1] == current_box_pos[1] and \
#        orig_obj_pos[2] == current_box_pos[2]:
#            effect = 'right'
##    elif round(orig_obj_pos[0],3) == round(current_box_pos[0] - 0.3,3) and \
#    elif round(current_box_pos[0],3) < round(orig_obj_pos[0],3) and \
#        orig_obj_pos[1] == current_box_pos[1] and \
#        orig_obj_pos[2] == current_box_pos[2]:
#            effect = 'left'
##        round(orig_obj_pos[1],3) == round(current_box_pos[1] + 0.3,3) and \
#    elif orig_obj_pos[0] == current_box_pos[0] and \
#        round(current_box_pos[1],3) > round(orig_obj_pos[1],3) and \
#        orig_obj_pos[2] == current_box_pos[2]:
#            effect = 'far'
#    elif orig_obj_pos[0] == current_box_pos[0] and \
#        round(current_box_pos[1],3) < round(orig_obj_pos[1],3) and \
#        orig_obj_pos[2] == current_box_pos[2]:
#            effect = 'close'
#    else:
#        print('Effect discretization failed for', orig_obj_pos,
#              'regarding the initial pos', current_box_pos)        
#    return effect

#'''
#Given a final position identify the effect
#Virtual coordinates
#'''
#def compute_effect(initial_obj_pos, 
#                   final_obj_pos):
##    print(initial_obj_pos, final_obj_pos)
#    if final_obj_pos[0:2] == \
#        [round(initial_obj_pos[0] + sim_param.obj_displacement, sim_param.round_value), 
#         initial_obj_pos[1]]:
#        return 'far'        
#    elif final_obj_pos[0:2] == \
#        [round(initial_obj_pos[0] - sim_param.obj_displacement, sim_param.round_value), 
#         initial_obj_pos[1]]:
#        return 'close'
#    elif final_obj_pos[0:2] == \
#        [initial_obj_pos[0], 
#         round(initial_obj_pos[1] + sim_param.obj_displacement, sim_param.round_value)]:
#        return 'left'        
#    elif final_obj_pos[0:2] == \
#        [initial_obj_pos[0], 
#         round(initial_obj_pos[1] - sim_param.obj_displacement, sim_param.round_value)]:
#        return 'right'
#    else:
#        print('ERROR - compute_effect : no effect identified')
#        return ''
        
'''
Given a final position identify the effect
Real coordinates
'''
def compute_effect(initial_obj_pos, final_obj_pos):
    delta_x = round(final_obj_pos[0] - initial_obj_pos[0], sim_param.round_value)
    delta_y = round(final_obj_pos[1] - initial_obj_pos[1], sim_param.round_value)
    
    if delta_y > 0:
        print('\ndiff', abs(round(delta_x/delta_y)))
    
    if delta_x == 0 and delta_y == 0:
#        print('ERROR - compute_effect :', 
#              initial_obj_pos,
#              final_obj_pos)
        return ''
    else:     
        if delta_x > sim_param.effect_validation*delta_y: ## far or close
            if final_obj_pos[0] > initial_obj_pos[0] :
                return 'far'
            elif final_obj_pos[0] < initial_obj_pos[0] :    
                return 'close'
            else:
                return ''
                
        elif delta_x*sim_param.effect_validation < delta_y: ## left or right
            if final_obj_pos[1] > initial_obj_pos[1] :
                return 'left'
            elif final_obj_pos[1] < initial_obj_pos[1] :
                return 'right'
            else:
                return ''
        
        elif abs(round(delta_x/delta_y, sim_param.round_value)) > 0.5: ## diagonals

            if delta_x > 0 and delta_y > 0: 
                return 'far_left'
            elif delta_x > 0 and delta_y < 0: 
                return 'far_right'
            elif delta_x < 0 and delta_y > 0: 
                return 'close_left'
            elif delta_x < 0 and delta_y < 0: 
                return 'close_right'
            else:
                return ''                  
 
        else:
            return ''
            
        
            
#'''
#Given a final position identify the effect
#Real coordinates
#'''
#def compute_effect(initial_obj_pos, final_obj_pos):
#    delta_x = round(abs(final_obj_pos[0] - initial_obj_pos[0]), sim_param.round_value)
#    delta_y = round(abs(final_obj_pos[1] - initial_obj_pos[1]), sim_param.round_value)
#    
#    mov_axis = [1 if curr_diff > 0.001 else 0 for curr_diff in [delta_x, delta_y]]
#
#    ############ far-close
#    value_x = 'zero'
#    ## if movement in X
#    if mov_axis[0]:
#        if initial_obj_pos[0] > final_obj_pos[0]:            
#            value_x = 'close'
#        else:
#            value_x = 'far'
#    
#    ############ left-right
#    value_y = 'zero'
#    ## if movement in y
#    if mov_axis[1]:
#        if initial_obj_pos[1] > final_obj_pos[1]:            
#            value_y = 'right'
#        else:
#            value_y = 'left'
#                
#    effect_found = ''        
#    if value_x != 'zero':
#        effect_found += value_x + '_'
#    if value_y != 'zero':
#        effect_found += value_y + '_'  
#    
#    ## remove last 
#    if effect_found != '' and effect_found[-1] == '_':
#        effect_found = effect_found[:-1]
#
##    print ('effect_found :', effect_found)
#
#    return effect_found
            
if __name__ == '__main__':
    print(compute_effect([0,0,0], [1,1,1]))
    print(compute_effect([0,0,0], [-1,1,1]))
    print(compute_effect([0,0,0], [1,-1,1]))
    print(compute_effect([0,0,0], [-1,-1,1]))
    
    print(compute_effect([0,0,0], [1,0,1]))
    print(compute_effect([0,0,0], [0,1,-1]))
    print(compute_effect([0,0,0], [-1,0,1]))
    print(compute_effect([0,0,0], [0,-1,-1]))
    
    print(compute_effect([0,0,0], [0,0,0]))
           

