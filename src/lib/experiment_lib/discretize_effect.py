# -*- coding: utf-8 -*-
"""
@author: maestre
"""

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
    delta_x = abs(final_obj_pos[0] - initial_obj_pos[0])
    delta_y = abs(final_obj_pos[1] - initial_obj_pos[1])
    
    if delta_x == 0 and delta_y == 0:
#        print('ERROR - compute_effect :', 
#              initial_obj_pos,
#              final_obj_pos)
        return None
    else:        
        if delta_x > sim_param.effect_validation*delta_y: ## far or close
            if final_obj_pos[0] > initial_obj_pos[0] :
                return 'far'
            elif final_obj_pos[0] < initial_obj_pos[0] :    
                return 'close'
        elif delta_x*sim_param.effect_validation < delta_y: ## left or right
            if final_obj_pos[1] > initial_obj_pos[1] :
                return 'left'
            elif final_obj_pos[1] < initial_obj_pos[1] :
                return 'right'
            else:
                return None

    
'''
Test
'''
if __name__ == '__main__':
#    print(compute_effect([0.3, 0.3, 0]))
    
#    print(compute_effect([0.65,-0.05,-0.09], new_pos[1]))
#    print(compute_effect([-0.3,0,0]))
    
    print(compute_effect([0.65,-0.1,-0.09], [0.95,-0.1,-0.09])) ## far
    print(compute_effect([0.65,-0.1,-0.09], [0.35,-0.1,-0.09])) ## close
    print(compute_effect([0.65,-0.1,-0.09], [0.65,0.2,-0.09])) ## left
    print(compute_effect([0.65,-0.1,-0.09], [0.65,-0.4,-0.09])) ## right    
    
#    print(compute_effect_3d([0.65,-0.1,-0.09], [0.95,-0.1,-0.09])) ## far
#    print(compute_effect_3d([0.65,-0.1,-0.09], [0.35,-0.1,-0.09])) ## close
#    print(compute_effect_3d([0.65,-0.1,-0.09], [0.65,0.2,-0.09])) ## left
#    print(compute_effect_3d([0.65,-0.1,-0.09], [0.65,-0.4,-0.09])) ## right       

