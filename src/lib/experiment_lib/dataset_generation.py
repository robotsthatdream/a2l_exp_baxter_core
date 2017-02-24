#!/usr/bin/env python

"""
@author: maestre
"""
from __future__ import print_function

import random
#import numpy as np

import os, sys
run_path = os.path.realpath(os.path.abspath(os.path.join('..', '..')))
sys.path.append(run_path)
lib_path = os.path.realpath(os.path.abspath(os.path.join('..', 'a2l_core_lib')))
sys.path.append(lib_path)
import dataset_directed_extended as dde
import dataset_random as dr
import environment_dynamics as env
import environment_delta as delta
import discretize_effect as discr
import inference_ros as infer_ros
import inf_traj_series_dataset_classes as dataset_series_classes
import simulation_parameters as sim_param

if sim_param.print_discr_random_dataset:
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    import environment_setup as setup
    
import scipy.spatial.distance as d

'''
Create directed or random dataset
'''
def create_dataset(dataset_type, nb_initial_pos):
#    if dataset_type == 'directed':
#        print('DIRECTED dataset generated')
#        return dd.create_discr_trajs(nb_initial_pos)        
#    el
    if dataset_type == 'directed':
        print('DIRECTED dataset generated')
        return dde.create_discr_trajs(nb_initial_pos)        
        
    elif dataset_type == 'random':
        print('RANDOM dataset generated')
        return dr.create_discr_trajs(nb_initial_pos)        
    else:
        print('ERROR - create_dataset - wrong dataset_type value')           
        sys.exit()  


'''
Read trajs of externally generated dataset
traj = [EP EO OP1 OO1 OP2 OO2 etc]
EP = x y z
EO = roll pitch yaw
OPi = x y z
OOi = roll pitch yaw
'''
def read_dataset(filename):
    mean_move_size = 0
    nb_moves = 0
    delta_vector = []
    lines = open(filename, 'r').readlines()
    for line in lines:        
        current_delta_vector = []
        pos_rot_vector = line[:-2].split(',') ## remove final , and EOL
        nb_obj = len(sim_param.obj_name_vector) ## expected
        nb_dataset_obj = sim_param.dataset_nb_objects ## actual
        
        obj_initial_pos = [float(pos_rot_vector[6]),
                           float(pos_rot_vector[7]),
                           float(pos_rot_vector[8])]
        obj_final_pos = [float(pos_rot_vector[-6 -6*(nb_obj-1)]),
                         float(pos_rot_vector[-5 -6*(nb_obj-1)]),
                         float(pos_rot_vector[-4 -6*(nb_obj-1)])]
        obtained_effect = env.identify_effect_3d(obj_initial_pos,
                                                 obj_final_pos)
        
        related_info_size = 6 + 6*nb_obj
        for pos in range(0, len(pos_rot_vector)-related_info_size, related_info_size):
            current_x = float(pos_rot_vector[pos+0])
            current_y = float(pos_rot_vector[pos+1])
            current_z = float(pos_rot_vector[pos+2])
            
            next_x = float(pos_rot_vector[pos+related_info_size+0])
            next_y = float(pos_rot_vector[pos+related_info_size+1])
            next_z = float(pos_rot_vector[pos+related_info_size+2])
            
            mean_move_size += d.euclidean([current_x, current_y, current_z],
                                          [next_x, next_y, next_z])
            nb_moves += 1

            current_next_obj_pos_vector = []
            for curr_obj_id in range(1,len(sim_param.obj_name_vector)+1):
                
                current_obj_pos_x = float(pos_rot_vector[pos+6*curr_obj_id+0])
                current_obj_pos_y = float(pos_rot_vector[pos+6*curr_obj_id+1])
                current_obj_pos_z = float(pos_rot_vector[pos+6*curr_obj_id+2])
                
                next_obj_pos_x = float(pos_rot_vector[pos+related_info_size+6*curr_obj_id+0])
                next_obj_pos_y = float(pos_rot_vector[pos+related_info_size+6*curr_obj_id+1])
                next_obj_pos_z = float(pos_rot_vector[pos+related_info_size+6*curr_obj_id+2])            

                current_next_obj_pos = [current_obj_pos_x, current_obj_pos_y,current_obj_pos_z,
                                        next_obj_pos_x, next_obj_pos_y, next_obj_pos_z]
                current_next_obj_pos_vector.append(current_next_obj_pos)
                
            current_delta = delta.Delta(
                obtained_effect,
                current_x,current_y,current_z,
                next_x, next_y,next_z,
                current_next_obj_pos_vector)
                                
#                current_obj_pos_x, current_obj_pos_y,current_obj_pos_z,
#                next_obj_pos_x, next_obj_pos_y, next_obj_pos_z,
#                True) ## Extended trajs are always moving the obj            
            current_delta_vector.append(current_delta)
        delta_vector += current_delta_vector
        
    mean_move_size /= nb_moves
    sim_param.step_length = round(mean_move_size, sim_param.round_value)
    print('Move step length:', sim_param.step_length)
    return delta_vector

'''
Compute cumulated performance value for each dataset type
'''
def compute_cumulated_perf(current_dataset,
                           #dataset_stats_vector,
                           learn_algo_vector,
                           initial_pos_vector):
    dataset_perf_value_vector = []
#    for current_dataset in dataset_stats_vector:

    ## Classes to store the inferred trajs cumulated perf   
    dataset_name = current_dataset.get_dataset_name()
    current_dataset_cumulated_perf_class = \
        dataset_series_classes.Dataset_stats(dataset_name)
    for current_algo in learn_algo_vector:
        current_perf_value_dict = \
            infer_ros.compute_perf_value_for_init_pos(
                    current_dataset,
                    current_algo,
                    initial_pos_vector)  
        current_dataset_cumulated_perf_class.add_algo_results(
            current_algo, current_perf_value_dict)
    ## Store them
    dataset_perf_value_vector.append(
        current_dataset_cumulated_perf_class)    
    return dataset_perf_value_vector

'''
Extend dataset generating new deltas based on successful trajs
'''
def extend_dataset(current_dataset_stats, 
                   current_nb_initial_pos, ## 8
                   learn_algo_vector,
                   perf_value_changes_vector, ## [0, 3, 1, 0])
                   dataset_size):
    
    for current_algo in learn_algo_vector:
#        print('\n',current_algo.upper())
        current_algo_stats_class = \
            current_dataset_stats.get_algo_results(current_algo)
        current_algo_score = current_algo_stats_class.get_algo_score()
            
        current_dataset_size_dict = \
            current_algo_stats_class.get_results_dict()
        current_dataset_size_class = \
            current_dataset_size_dict[current_nb_initial_pos]
        current_global_perf = \
            current_dataset_size_class.get_global_performance_value()
        current_trajs_dict = \
            current_dataset_size_class.get_indiv_traj_info()
            
        ## for each initial position pos and effect generate new trajs
        new_delta_vector = []
        new_random_directed_trajs_vector = []
        for current_init_pos in range(current_nb_initial_pos):
            for effect in sim_param.effect_values:
#                print('\n',current_init_pos, effect)
                new_random_directed_trajs = False

                current_traj_class = \
                    current_trajs_dict[current_init_pos, effect]
                current_traj_res = current_traj_class.get_traj_res()
                
                ## if a perf value is static for long time, generate random trajs
                effect_pos = -1
                if effect == 'up':
                    effect_pos = 0
                elif effect == 'left':
                    effect_pos = 1
                elif effect == 'down':
                    effect_pos = 2
                else:
                    effect_pos = 3
                
                perf_value_pos = current_init_pos * \
                                 len(sim_param.effect_values) + \
                                 effect_pos                                
                if perf_value_changes_vector[perf_value_pos] == \
                   sim_param.max_repeated_perf_value:
                    new_random_directed_trajs = True
                    perf_value_changes_vector[perf_value_pos] = 0
                    new_random_directed_trajs_vector.append(
                        [current_init_pos,
                        effect])
                    print("DIRECTED RANDOM TRAJS FOR VECTOR POS", 
                          perf_value_pos,
                          'init_pos - effect :',
                          current_init_pos, effect)                    
                
                ## generate new trajs from the scratch 
                ## for a specific effect (if True)
                if new_random_directed_trajs:
                    current_traj = \
                        current_traj_class.get_eef_traj_vector()
                    current_delta = \
                        current_traj_class.get_delta_vector()                    
                    current_new_delta_vector = \
                        extend_trajectory(current_traj, 
                                          current_delta,
                                          current_nb_initial_pos,
                                          current_init_pos, 
                                          effect,
                                          new_random_directed_trajs,
                                          dataset_size)
                    print('len new deltas', len(current_new_delta_vector),
                          'init_pos - effect :',
                          current_init_pos, effect)
                    new_delta_vector = \
                        new_delta_vector + current_new_delta_vector
                        
                else:
#                    print('REGULAR RANDOM TRAJS FOR init_pos - effect :',
#                          current_init_pos, effect)
                    ## or generate them based on previous contacts (if False)
                    if current_traj_res == 'success'or current_traj_res == 'false_pos':
                        current_traj = \
                            current_traj_class.get_eef_traj_vector()
                        current_delta = \
                            current_traj_class.get_delta_vector()                    
                        current_new_delta_vector = \
                            extend_trajectory(current_traj, 
                                              current_delta,
                                              current_nb_initial_pos,
                                              current_init_pos, 
                                              effect,
                                              new_random_directed_trajs,
                                              dataset_size)
#                        print('len new deltas', len(current_new_delta_vector),
#                              'init_pos - effect :',
#                              current_init_pos, effect)
                        new_delta_vector = \
                            new_delta_vector + current_new_delta_vector
#    print('total delta extension :', len(new_delta_vector))
    return new_delta_vector, new_random_directed_trajs_vector

   

'''
Given a trajectory (init_pos-effect) tries to extend it with new moves
'''
def extend_trajectory(current_traj_vector, ## [WPs]
                      current_delta_vector, ## [delta_class]
                      nb_initial_pos, ## 8
                      current_init_pos,  ## 0,1,2,3,..., 7
                      orig_effect,
                      new_random_directed_trajs, ## bool
                      dataset_size):

    ## OPTION 1) generate specific trajs for a desired effect
    if new_random_directed_trajs:
        return dr.create_effect_trajs(nb_initial_pos, 
                                      current_init_pos,
                                      orig_effect,
                                      dataset_size)
    
    ## OPTION 2) generate random trajs for all the effects
    step_length = sim_param.step_length
    step_options = [-step_length, 0, step_length]
    new_delta_trajs_vector = []
    nb_trajs_found = 0    
    
    ## generate max N new trajs        
    nb_new_traj = 0
    while nb_new_traj < sim_param.extend_max_trajs:
        
        ## for (almost) each WP of the traj        
#        for curr_wp in range(1,len(current_traj_vector)):
        curr_wp = 1
        while curr_wp < len(current_traj_vector):
            ## new tmp traj from the beginning to this WP            
            tmp_traj = current_traj_vector[:-curr_wp]
            tmp_delta = current_delta_vector[:-curr_wp]
            
            traj_vector_x = [v[0] for v in tmp_traj]
            traj_vector_y = [v[1] for v in tmp_traj]
            nb_new_delta = 0
            obj_moved = False
            extended_delta_vector = tmp_delta
            effect = 'None'
            
            ## extend with new random moves from the closest to the more far one
            ## the more far is the WP respect the object, the more deltas can generate
            while nb_new_delta < (sim_param.extend_max_movs + (curr_wp-1)) and \
                  not obj_moved:

                current_x = traj_vector_x[-1]
                current_y = traj_vector_y[-1]
                if sim_param.semi_random_trajs:
                    new_x = current_x + random.choice(step_options)
                    new_y = current_y + random.choice(step_options)
                else:
                    new_x = current_x + random.uniform(-step_length*2,step_length*2)
                    new_y = current_y + random.uniform(-step_length*2,step_length*2)                
                traj_vector_x.append(new_x)
                traj_vector_y.append(new_y)
                
                ## compute new box pos
                updated_obj_pos = env.compute_obj_pos(
                                        [current_x,current_y,0],
                                        [new_x, new_y,0])             
                        
                ## store current delta if there was a move
                if current_x != new_x or \
                    current_y != new_y:
                        
                    current_delta = delta.Delta(
                                effect,
                                current_x,current_y,0,
                                new_x, new_y,0,
                                sim_param.obj_pos[0], sim_param.obj_pos[1],0,
                                updated_obj_pos[0], updated_obj_pos[1],0,
                                obj_moved)
                    extended_delta_vector.append(current_delta)
                
                ## check if the obj was moved
                obj_moved = True \
                    if updated_obj_pos != sim_param.obj_pos else False                                    

                #############################################################
                if False and obj_moved:   
                    print(current_init_pos, orig_effect, 
                          "- WP", curr_wp, "- extension", nb_new_delta)
                    
                    # plot new traj
                    import matplotlib.pyplot as plt
                    from matplotlib.patches import Rectangle
                    from matplotlib import gridspec
                    import environment_setup as setup
                    fig = plt.gcf()
                    fig.set_size_inches(8, 9)
                    gs = gridspec.GridSpec(1, 1)
                    ax = plt.subplot(gs[0])
                    
                    # set axis limits
                    ax.set_xlim(-1.5,1.5)
                    ax.set_ylim(-1.5,1.5)
                    
                    ## plot object
                    obj=Rectangle((0 - sim_param.obj_side/2, 
                                        0 - sim_param.obj_side/2), 
                                        sim_param.obj_side,sim_param. obj_side, 
                                        fc='grey')
                    ax.add_patch(obj)
                    ## plot the big circle points         
                    list_x_axis, list_y_axis, list_z_axis = \
                        setup.gen_init_eef(nb_initial_pos)
                    ax.plot(list_x_axis,list_y_axis,'.',c='grey')   
                    
                    ## read traj values
                    eef_traj = []
                    eef_traj.append([extended_delta_vector[0].get_wp_init().get_x(),
                                     extended_delta_vector[0].get_wp_init().get_y()])
                    for d in extended_delta_vector:
                        eef_traj.append([d.get_wp_final().get_x(),
                                        d.get_wp_final().get_y()])
                    
                    
                    for i in range(len(eef_traj)-1):
                        ## plot the segment
                        ax.plot([eef_traj[i][0],eef_traj[i+1][0]],
                                [eef_traj[i][1],eef_traj[i+1][1]],
                                '-*',
                                linewidth=2)
                                
                    plt.show()
                                
                    ##########################################################
                            
                nb_new_delta += 1
                ## end generate new deltas
            curr_wp += 1
                            
            ## compute new effect and store traj
            if obj_moved:                            
                nb_new_traj += 1
                nb_trajs_found += 1

                ## compute related effect
                obtained_effect = discr.compute_effect(updated_obj_pos)
#                print(obtained_effect)
                for d in extended_delta_vector:
                    d.set_effect(obtained_effect)
 
                if sim_param.only_store_different_effects:
                    if obtained_effect != orig_effect:
    #                    print(current_init_pos, orig_effect, '->', effect)
    #                    print(len(new_delta_trajs_vector), '->', 
    #                          len(extended_delta_vector))
                        new_delta_trajs_vector =\
                            new_delta_trajs_vector + extended_delta_vector
                else:
                    new_delta_trajs_vector =\
                        new_delta_trajs_vector + extended_delta_vector
            ## end generate traj

#    print (nb_trajs_found, "new trajs found extending", current_init_pos, orig_effect)
    return new_delta_trajs_vector
    
'''
if there is NOT success for the inference of a (init_pos, effect) then
if the perf value of the (init_pos, effect) changes between two iterations its value
comes to 0, else 1 is added
'''
def update_perf_value_changes(
            perf_value_changes_vector,
            prev_iter_dataset_stats_class_perf_value_vector,
            current_iter_dataset_stats_class_perf_value_vector):
    for pos in range(len(prev_iter_dataset_stats_class_perf_value_vector)):
        if current_iter_dataset_stats_class_perf_value_vector[pos] == 'success':
            perf_value_changes_vector[pos] = -1
        else:
            if prev_iter_dataset_stats_class_perf_value_vector[pos] != \
                current_iter_dataset_stats_class_perf_value_vector[pos]:            
                perf_value_changes_vector[pos] = 0
            else:
                perf_value_changes_vector[pos] = perf_value_changes_vector[pos] + 1

    return perf_value_changes_vector    

'''
Remove trajs from figure
'''
def reset_plot(nb_initial_pos):
    fig = plt.figure()
    fig.set_size_inches(7, 7)
    ax = fig.add_axes([0, 0, 1, 1])
    
    # set axis limits
    plt.xlim(-1.2,1.2)
    plt.ylim(-1.2,1.2)
#        ax.axis('off')
 
    # plot the origin
    origin = Rectangle((sim_param.obj_pos[0]-sim_param.obj_side/2, 
                        sim_param.obj_pos[1]-sim_param.obj_side/2), 
                       sim_param.obj_side, sim_param.obj_side, fc="grey")
    ax.add_patch(origin)
     
    ## plot the big circle points
    list_x_axis, list_y_axis, list_z_axis = \
        setup.gen_init_eef(nb_initial_pos)
    ax.plot(list_x_axis,list_y_axis,'o',c='r')

    return ax


''' 
Compute the perf value for each init pos and effect
'''
def plot_infer_trajs_iterations_effect_init_pos(iterations_dict,
                                                nb_init_pos,
                                                current_algo):
    current_iter_class = iterations_dict[nb_init_pos]
    current_random_data_stat_class = \
        (current_iter_class.get_dataset_stats_vector())[0] ## only the random dataset
    current_dataset_dict = current_random_data_stat_class.get_results_dict()       
    current_algo_class = current_dataset_dict[current_algo]
    current_algo_dict = current_algo_class.get_results_dict()
    current_dataset_size_class = current_algo_dict[sim_param.nb_init_pos_for_adaption]
    current_dataset_size_dict = current_dataset_size_class.get_indiv_traj_info()
    init_pos_results_vector = [] ## to store all the results in an iteration
    for current_init_pos in range(sim_param.nb_init_pos_for_adaption):
        init_pos_results = [] ## up, left, down, right
        for ef in ['up', 'left', 'down', 'right']:
            current_traj_class = \
                current_dataset_size_dict[current_init_pos, ef]
            current_effect_result = current_traj_class.get_traj_res()
            init_pos_results.append(current_effect_result)
            if current_traj_class.get_desired_effect() != \
               current_traj_class.get_obtained_effect():
                   print('Wrong effect for', 
                         current_init_pos, current_traj_class.get_desired_effect(),
                         ' instead', current_traj_class.get_obtained_effect())
        init_pos_results_vector = init_pos_results_vector + init_pos_results
    return init_pos_results_vector

'''
Test
'''

if __name__ == '__main__':
    tmp_traj = [[0.4, 0], 
                [0.3, 0.1], 
                [0.2, 0.2]]
    tmp_delta = [['orien_5','dist_0','down'], ## should be delta classes
             ['orien_4','dist_0','up'], ## vectors just to run the example
             ['orien_5','dist_0','left-down']]
    new_delta_trajs_vector = extend_trajectory(tmp_traj, tmp_delta, 8, 7, "down")
    
    
    
#    for extended_delta_vector in new_delta_trajs_vector :
#        for new_delta in extended_delta_vector:
    for new_delta in new_delta_trajs_vector:
            new_delta.print_me()