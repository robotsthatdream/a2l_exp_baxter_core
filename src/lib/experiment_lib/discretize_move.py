# -*- coding: utf-8 -*-
"""
Created on Thu Jul  7 16:54:57 2016

@author: maestre
"""
import os, sys
run_path = os.path.abspath(os.path.join('..', '..'))
sys.path.append(run_path)
import simulation_parameters as sim_param

'''
Given two positions return the related move
'''
def compute_move_discr(pos_init_vector, pos_final_vector):
    ## zero means both positions are similar
    same_value_threshold = sim_param.same_move_value

    
    ############ far-close
    value_x = 'zero'
    ## if movement in X
    if abs(pos_init_vector[0] - pos_final_vector[0]) > \
       same_value_threshold:
        if pos_init_vector[0] > pos_final_vector[0]:            
            value_x = 'close'
        else:
            value_x = 'far'
    
    ############ left-right
    value_y = 'zero'
    ## if movement in X
    if abs(pos_init_vector[1] - pos_final_vector[1]) > \
       same_value_threshold:
        if pos_init_vector[1] > pos_final_vector[1]:            
            value_y = 'right'
        else:
            value_y = 'left'
            
    ############ up-down
    value_z = 'zero'
    ## if movement in Z
    if abs(pos_init_vector[2] - pos_final_vector[2]) > \
       same_value_threshold:
        if pos_init_vector[2] > pos_final_vector[2]:            
            value_z = 'down'
        else:
            value_z = 'up'
            
    final_move = ''
    if value_x == 'zero' and value_y == 'zero' and value_z == 'zero':
        return 'zero'
        
    if value_x != 'zero':
        final_move += value_x + '_'
    if value_y != 'zero':
        final_move += value_y + '_'    
    if value_z != 'zero':
        final_move += value_z + '_'    
        
    if final_move[-1] == '_':
        final_move = final_move[:-1]

    return final_move
            
if __name__ == '__main__':
    print(compute_move_discr([0,0,0], [1,1,1]))

