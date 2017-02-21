# -*- coding: utf-8 -*-
"""
@author: maestre
"""
from __future__ import print_function

#import scipy.spatial.distance as spatial
import matplotlib.pyplot as plt

import os, sys
run_path = os.path.realpath(os.path.abspath(os.path.join('..', '..')))
sys.path.append(run_path)
import simulation_parameters as sim_param
import math

import discretize_move as discr_move
if sim_param.discr_hand_coded:
    import discretize_orientation_hand_coded as discr_orien
    import discretize_distance_hand_coded as discr_dist
else:
    import discretize_orientation_sections as discr_orien
    import discretize_distance_sections as discr_dist
    import discretize_inclination_sections as discr_inclin
    

'''
Compute distance discretization
'''
def compute_distance_discr():
    if sim_param.discr_hand_coded:
        return None
    else:
        if sim_param.experiment_type == 'a2l_dataset_extension':
            discr_sections = discr_dist.Sections( sim_param.obj_side/2,
                                          sim_param.circumference_radio,
                                  sim_param.nb_min_distance_sections)
        else:
            discr_sections = discr_dist.Sections(0,1,
                                  sim_param.nb_min_distance_sections)            
        if sim_param.debug:
            discr_sections.print_me()
        
        return discr_sections    
    
'''
Compute orientation discretization
'''
def compute_orientation_discr():
    if sim_param.discr_hand_coded:
        return None
    else:        
        orien_sections = discr_orien.Sections(sim_param.orien_min_angle, 
                                              sim_param.orien_max_angle,
                                              sim_param.nb_min_orientation_sections)                              
        if sim_param.debug:
            orien_sections.print_me('radians')
        
        return orien_sections          

'''
Compute inclination discretization
'''
def compute_inclination_discr():
    if sim_param.discr_hand_coded:
        return None
    else:
        inclin_sections = \
            discr_inclin.Sections(sim_param.inclin_min_angle, 
                                  sim_param.inclin_max_angle,
                                  sim_param.nb_min_inclination_sections)                         
        if sim_param.debug:
            inclin_sections.print_me('radians')
        
        return inclin_sections
    
'''
Discretize trajectories
'''
def discretize_trajs(delta_vector,
                     current_orien_discr, 
                     current_inclin_discr,
                     current_dist_discr):    
    discr_dataset_vector = []
    nb_delta = 0
    while nb_delta < len(delta_vector)-1:
        current_delta = delta_vector[nb_delta]
#        print('\n\nnb_delta', nb_delta)        
#        current_delta.print_me()

        
        ''' Compute move '''
        move = discr_move.compute_move_discr(
                     [current_delta.get_wp_init().get_x(),
                     current_delta.get_wp_init().get_y(),
                     current_delta.get_wp_init().get_z()],
                     [current_delta.get_wp_final().get_x(),
                     current_delta.get_wp_final().get_y(),
                     current_delta.get_wp_final().get_z()])

        if move != 'zero':
            discr_delta_values = [current_delta.get_effect(), move]        
            for obj_id in range(len(sim_param.obj_name_vector)):
                
                ''' Compute orientation ''' 
                orientation = discr_orien.compute_orientation_discr(
                                 [current_delta.get_wp_init().get_x(),
                                 current_delta.get_wp_init().get_y()],
                                 current_delta.get_obj_init(obj_id),
    #                             [current_delta.get_obj_init().get_x(),
    #                             current_delta.get_obj_init().get_y()],
                                 current_orien_discr)                                 
                ''' Compute inclination ''' 
                inclination = discr_inclin.compute_inclination_discr(
#                                 [current_delta.get_obj_init().get_x(),
#                                 current_delta.get_obj_init().get_y(),
#                                 current_delta.get_obj_init().get_z()],
                                 current_delta.get_obj_init(obj_id),                
                                 [current_delta.get_wp_init().get_x(),
                                 current_delta.get_wp_init().get_y(),
                                 current_delta.get_wp_init().get_z()],                             
                                 current_inclin_discr)    
                ''' Compute distance '''
                distance = discr_dist.compute_distance(
                             [current_delta.get_wp_init().get_x(),
                             current_delta.get_wp_init().get_y(),
                             current_delta.get_wp_init().get_z()],
#                             [current_delta.get_obj_init().get_x(),
#                             current_delta.get_obj_init().get_y(),
#                             current_delta.get_obj_init().get_z()],
                             current_delta.get_obj_init(obj_id),                             
                             current_dist_discr)            
                discr_delta_values = discr_delta_values + [distance, orientation, inclination]
                                          
#            print([current_delta.get_effect(), 
#                   move,
#                   distance,
#                   orientation,
#                   inclination])
#            discr_dataset_vector.append([current_delta.get_effect(), 
#                                         orientation,
#                                         inclination,
#                                         move,
#                                         distance])
        
#        print(discr_delta_values)
        discr_dataset_vector.append(discr_delta_values)
        nb_delta += 1    
    
    return discr_dataset_vector

'''
Save the discretized deltas composing the trajectories
'''
def save_discr_deltas(dataset_filename, discr_delta_vector):
    ''' Write the trajs discretized dataset '''
    file = open(dataset_filename, 'w')    
    file.write('effect')
    file.write(',')
    file.write('move')
    file.write(',')
    
    for obj_id in range(len(sim_param.obj_name_vector)):
        file.write('distance'+str(obj_id))
        file.write(',')
        file.write('orientation'+str(obj_id))
        file.write(',')
        file.write('inclination'+str(obj_id))
        if obj_id != len(sim_param.obj_name_vector)-1:
            file.write(',')
    file.write('\n')     
    
    traj_pos = 0
    for discr_delta in discr_delta_vector:
        for nb_value in range(len(discr_delta)):
            file.write(discr_delta[nb_value])
            if nb_value != (len(discr_delta)-1):
                file.write(',')
        file.write('\n')                   
        traj_pos += 1
    file.close()
        
    if sim_param.print_discr_random_dataset:
        plt.show()    
        
'''
Test
'''
if __name__ == '__main__':
    t = compute_distance_discr()
    t.print_me()
    print()
    
    t = compute_orientation_discr()
    t.print_me('radians')
    print()
    t.print_me('degrees')
    print()
    
    t = compute_inclination_discr()
    t.print_me('radians')
    print()
    t.print_me('degrees')    