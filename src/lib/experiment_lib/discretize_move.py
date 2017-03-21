# -*- coding: utf-8 -*-
"""
Created on Thu Jul  7 16:54:57 2016

@author: maestre
"""
from __future__ import print_function

import os, sys
run_path = os.path.realpath(os.path.abspath(os.path.join('..', '..')))
sys.path.append(run_path)
import simulation_parameters as sim_param

'''
Given two positions return the related move
'''
def compute_move_discr(pos_init_vector, pos_final_vector):

    diff_x = abs(pos_init_vector[0] - pos_final_vector[0])
    diff_y = abs(pos_init_vector[1] - pos_final_vector[1])
    diff_z = abs(pos_init_vector[2] - pos_final_vector[2])
#    print('Diffs :', diff_x, diff_y, diff_z)

    if sim_param.semi_random_trajs:
        mov_axis = [1 if curr_diff > 0.001 else 0 for curr_diff in [diff_x, diff_y, diff_z]]
    else:
        ## select max dist
        max_diff = max(diff_x, diff_y, diff_z)
        if max_diff < 0.0001:
            return 'zero'
    
        ## if others are similar lets take them into account in the move
        mov_axis = [False, False, False]
        pos = 0
        for curr_diff in [diff_x, diff_y, diff_z]:
            if curr_diff/max_diff > 0.5:
                mov_axis[pos] = True
            pos += 1
        
    ############ far-close
    value_x = 'zero'
    ## if movement in X
    if mov_axis[0]:
        if pos_init_vector[0] > pos_final_vector[0]:            
            value_x = 'close'
        else:
            value_x = 'far'
    
    ############ left-right
    value_y = 'zero'
    ## if movement in y
    if mov_axis[1]:
        if pos_init_vector[1] > pos_final_vector[1]:            
            value_y = 'right'
        else:
            value_y = 'left'
            
    ############ up-down
    value_z = 'zero'
#    ## if movement in Z
#    if mov_axis[2]:
#        if pos_init_vector[2] > pos_final_vector[2]:            
#            value_z = 'down'
#        else:
#            value_z = 'up'
                
    final_move = ''
    if value_x == 'zero' and value_y == 'zero' and value_z == 'zero':
        return 'zero'
        
    if value_x != 'zero':
        final_move += value_x + '_'
    if value_y != 'zero':
        final_move += value_y + '_'    
    if value_z != 'zero':
        final_move += value_z + '_'    
    
    ## remove last _
    if final_move[-1] == '_':
        final_move = final_move[:-1]

#    print ('final_move :', final_move)

    return final_move
            
if __name__ == '__main__':
    print(compute_move_discr([0,0,0], [1,1,1]))

