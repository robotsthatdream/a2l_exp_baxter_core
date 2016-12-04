# -*- coding: utf-8 -*-
"""
@author: maestre
"""
from __future__ import print_function

#import matplotlib.pyplot as plt
#from matplotlib.patches import Rectangle, Circle
#from matplotlib import colors, colorbar
#from matplotlib import gridspec
#from math import sin,cos
import numpy as np
import random
import pyAgrum as agrum
from collections import OrderedDict

import os, sys
run_path = os.path.abspath(os.path.join('..', '..'))
sys.path.append(run_path)
lib_path = os.path.abspath(os.path.join('..', 'a2l_core_lib'))
sys.path.append(lib_path)
import environment_dynamics as env
import environment_setup as setup
import environment_delta as delta
import ros_services
import inf_traj_series_dataset_classes as dataset_series_classes
import simulation_parameters as sim_param

if sim_param.discr_hand_coded:
    import discretize_orientation_hand_coded as discr_orien
    import discretize_distance_hand_coded as discr_dist
else:
    import discretize_orientation_sections as discr_orien
    import discretize_inclination_sections as discr_inclin
    import discretize_distance_sections as discr_dist

'''
Infere trajs for a dataset, algo and nb_init_pos, 
for each initial position of the eef and for each goal
'''
def infere_trajectories(current_results_folder,
                        bn, 
                        alg,
                        dataset_size_class,
                        dataset_string,
                        current_obj_pos,
                        nb_initial_pos,
                        nb_traj,
                        current_orien,
                        current_inclin,
                        current_dist):            
    ## to infer next move
    ie = agrum.LazyPropagation(bn) 
    
    ## create initial positions of eef
    _list_x_axis, _list_y_axis, _list_z_axis = \
        setup.gen_init_eef(nb_initial_pos)    
    init_pos_vector = list(
        zip(_list_x_axis, 
            _list_y_axis, 
            _list_z_axis))

    ## restart statistics
    nb_success_r = 0
    nb_success_l = 0
    nb_success_u = 0
    nb_success_d = 0
    nb_false_positive_r = 0
    nb_false_positive_l = 0
    nb_false_positive_u = 0
    nb_false_positive_d = 0
    nb_fail_r = 0
    nb_fail_l = 0
    nb_fail_u = 0
    nb_fail_d = 0
    
    mean_prob = 0
    
#    ## restart scenario
#    if sim_param.debug_infer:
#        print('CALL SERVICE restart_world')
#    success = ros_services.call_restart_world()
#    if not success:
#            print("ERROR - restart_world failed")
    
    ## for each initial position infere / plot / save a traj
    nb_init_pos = 0 ## pos in eef_pos_vector
    total_inferred_discr_delta_vector = [] ## delta knowledge created during evaluations
    for init_pos_coord in init_pos_vector:    
#    for eef_pos in eef_pos_vector[0:2]:
        nb_effect = 0
        
        ## for each effect
        while nb_effect < len(sim_param.effect_values):
            desired_effect = sim_param.effect_values[nb_effect]
            
            ## infere traj
            res, delta_nb_var, eef_traj, \
            delta_class_vector, obtained_effect, \
            tmp_inferred_discr_delta_vector = \
                infere_traj(bn, ie,
                            nb_init_pos,
                            init_pos_coord, ## coord XYZ
                            desired_effect, 
                            current_obj_pos,
                            current_orien, current_inclin, current_dist)

            ## store current inferred traj
            tmp_infer_traj_class = \
                dataset_series_classes.Traj_info(eef_traj,
                                                 delta_class_vector,
                                                 res, 
                                                 desired_effect, 
                                                 obtained_effect)
            dataset_size_class.add_inferred_traj(nb_init_pos, desired_effect, 
                                                 tmp_infer_traj_class)

            ## store delta know created during iteration
#            print("Inferred deltas for (", nb_effect, effect, ") : ", 
#                  len(tmp_inferred_discr_delta_vector))
            total_inferred_discr_delta_vector = \
                total_inferred_discr_delta_vector + tmp_inferred_discr_delta_vector

            ## compute current mean prob
            traj_mean_prob = 0
            for traj in eef_traj:
                traj_mean_prob += traj[3]
            traj_mean_prob = traj_mean_prob / len(eef_traj)
            mean_prob += traj_mean_prob
            
#            ## plot / save traj
#            if sim_param.save_trajs or \
#                sim_param.save_some_trajs or \
#                sim_param.plot_trajs or \
#                sim_param.plot_some_trajs:
#                tmp_plot_bool = True
#                
#                if sim_param.plot_some_trajs or sim_param.save_some_trajs:
#                    current_pos = \
#                        round((float(nb_eef_pos)/len(eef_pos_vector))*100)
#                    if not current_pos == 0 and \
#                        not current_pos == 75:
#                            tmp_plot_bool = False
#                
#                if tmp_plot_bool:
#                    filepath = current_results_folder + \
#                                dataset_string + '_' + \
#                                str(nb_traj) + '_' + \
#                                str(nb_initial_pos) + '_' + \
#                                alg + '_' + \
#                                str(nb_eef_pos) + '_' + \
#                                effect
#                    
#                    plot_save_infere_traj(filepath,
#                                          init_pos_coord, effect, 
#                                          delta_nb_var, eef_traj,
#                                          current_obj_pos,
#                                          res,
#                                          nb_initial_pos,
#                                          current_orien,current_dist)
            ## update statistics
            if res == 'success' and desired_effect == 'right':
                nb_success_r += 1
            elif res == 'success' and desired_effect == 'left':
                nb_success_l += 1
            elif res == 'success' and desired_effect == 'up':
                nb_success_u += 1
            elif res == 'success' and desired_effect == 'down':
                nb_success_d += 1
            elif res == 'false_pos' and desired_effect == 'right':
                nb_false_positive_r += 1
            elif res == 'false_pos' and desired_effect == 'left':
                nb_false_positive_l += 1                
            elif res == 'false_pos' and desired_effect == 'up':
                nb_false_positive_u += 1
            elif res == 'false_pos' and desired_effect == 'down':
                nb_false_positive_d += 1
            elif res == 'fail' and desired_effect == 'right':
                nb_fail_r += 1
            elif res == 'fail' and desired_effect == 'left':
                nb_fail_l += 1                
            elif res == 'fail' and desired_effect == 'up':
                nb_fail_u += 1
            elif res == 'fail' and desired_effect == 'down':
                nb_fail_d += 1
            nb_effect += 1

        nb_init_pos += 1
    
    if sim_param.print_stats:
        print('\nNumber of trajectories inferred :',  len(sim_param.effect_values) 
                                                * len(init_pos_vector))
        
        print('\nSuccess trajectories :', nb_success_r + nb_success_l +
                                                nb_success_u + nb_success_d)     
        
        print('\nFalse positive trajectories :', nb_false_positive_r + 
                                                nb_false_positive_l +
                                                nb_false_positive_u +
                                                nb_false_positive_d)
        
        print('\nFailed trajectories :', nb_fail_r + nb_fail_l +
                                                nb_fail_u + nb_fail_d)
    
    succ_value = sum([nb_success_r, nb_success_l, nb_success_u, nb_success_d]) 
    false_pos_value = sum([nb_false_positive_r, nb_false_positive_l,
                        nb_false_positive_u, nb_false_positive_d])
    fail_value = sum([nb_fail_r, nb_fail_l, nb_fail_u, nb_fail_d])
    
    mean_prob = mean_prob / (succ_value + false_pos_value + fail_value)
    dataset_size_class.set_inference_res([succ_value, 
                                          false_pos_value, 
                                          fail_value])
    dataset_size_class.set_mean_prob_move(mean_prob)
    
    return total_inferred_discr_delta_vector
      
'''
Infere new traj to get a desired effect
'''
def infere_traj(bn, ie, 
                nb_init_pos,
                init_pos_coord, # XYZ
                expected_effect, 
                initial_obj_pos,
                current_orien,
                current_inclin,
                current_dist):
    
    if sim_param.debug_infer:
        print('\n////////////////////////////////')
        print('NEW TRAJ for (init_pos, effect)', 
              nb_init_pos, expected_effect.upper())

    ''' Restart scenario '''         
    success = ros_services.call_restart_world()
#    if not success:
#            print("ERROR - restart_world failed")
    
    ''' Move eef to initial position '''     
    success = ros_services.call_move_to_initial_position(nb_init_pos)
#    if not success:
#            print("ERROR - move_to_initial_position failed")
    
    ''' Infere new trajectory '''      
    eef_pose = ros_services.call_get_eef_pose('left')      
    eef_traj = [[eef_pose[0], eef_pose[1], eef_pose[2], 0]]
    discr_traj = []
    delta_vector = [] ## [orientation, distance, next_mov_discr]
    delta_class_vector = []
    obj_moved = False
    i = 0
    delta_nb_var_res = -1
    traj_inferred_discr_delta_vector = []
    while not obj_moved and i < sim_param.inferred_max_moves:
        print('Inferred delta ', i)
        
        current_eef_x = eef_traj[-1][0] ## last value added
        current_eef_y = eef_traj[-1][1]
        current_eef_z = eef_traj[-1][2]
        
        ## compute current variables value
        orientation = discr_orien.compute_orientation_discr(
            [current_eef_x,current_eef_y], 
            initial_obj_pos,
            current_orien)
            
        inclination = discr_inclin.compute_inclination_discr(
            [current_eef_x,current_eef_y,current_eef_z], 
            initial_obj_pos,
            current_inclin)
        print("current inclination :", inclination)
                            
        distance = discr_dist.compute_distance(
            [current_eef_x,current_eef_y], 
            initial_obj_pos,
            current_dist)
        node_names = ['effect','orientation', 'inclination', 'distance']
        node_values = [expected_effect,orientation, inclination, distance]
            
        ## infere next move
        next_mov_discr, prob_value, delta_nb_var = \
            infere_mov(bn, ie,                                        
                       node_names,
                       node_values)                        
                       
        if delta_nb_var_res == -1:
           delta_nb_var_res = delta_nb_var 

        if sim_param.debug_infer:
            print(expected_effect.upper(),
                  orientation.upper(),
                  inclination.upper(),
                  distance.upper(), 
                  "-> ", 
                  next_mov_discr.upper(), 
                  "with probability", prob_value)
                
        delta_x = 0
        delta_y = 0
        delta_z = 0
        mov_step = sim_param.step_length
        if 'far' in next_mov_discr:
            delta_x = mov_step
        elif 'close' in next_mov_discr:
            delta_x = -mov_step
        if 'right' in next_mov_discr:
            delta_y = -mov_step
        elif 'left' in next_mov_discr:
            delta_y = mov_step
        if 'up' in next_mov_discr:
            delta_z = mov_step
        elif 'down' in next_mov_discr:
            delta_z = -mov_step
        print("DELTA :", delta_x, delta_y, delta_z)
              
        ## move eef to new position
        success = ros_services.call_execute_delta_motion(delta_x, 
                                                         delta_y, 
                                                         delta_z)
#        if not success:
#            print("ERROR - infere_traj : delta move failed")
                                                         
        eef_pose = ros_services.call_get_eef_pose('left')      
        next_eef_x = eef_pose[0]
        next_eef_y = eef_pose[1]
        next_eef_z = eef_pose[2]
              
        ## check if obj was moved
        current_obj_pos = [-1, -1, -1]
        current_obj_pos = ros_services.call_get_model_state(sim_param.obj_name)
#        print(current_obj_pos)
        obj_moved = (abs(initial_obj_pos[0] - current_obj_pos[0]) > 0.01) or \
                    (abs(initial_obj_pos[1] - current_obj_pos[1]) > 0.01)
        if obj_moved:
            if sim_param.debug_infer:
                print('OBJECT MOOOOOOOOOOVED!!!!')
                print(initial_obj_pos)
                print(current_obj_pos[0:3])
                print(abs(initial_obj_pos[0]-current_obj_pos[0]),
                      abs(initial_obj_pos[1]-current_obj_pos[1]),
                      abs(initial_obj_pos[2]-current_obj_pos[2]))

        ## store delta also as delta class              
        current_delta_class = delta.Delta(
                    expected_effect,
                    current_eef_x,current_eef_y,sim_param.eef_z_value,
                    next_eef_x, next_eef_y, next_eef_z,
                    initial_obj_pos[0], initial_obj_pos[1], initial_obj_pos[2],
                    current_obj_pos[0], current_obj_pos[1], current_obj_pos[2],
                    obj_moved)            
        delta_class_vector.append(current_delta_class)
    
        discr_traj.append(next_mov_discr)        
        eef_traj.append([next_eef_x,
                         next_eef_y, 
                         next_eef_z, 
                         prob_value])
        delta_vector.append([orientation, distance, next_mov_discr])
        
        i += 1                                                

    ''' Compute result '''
    res = ''
    obtained_effect = ''
    if not obj_moved:
        if sim_param.debug_infer:
            print('OBJECT nottttttttttttt moved!!!!')
        res = 'fail'
        traj_inferred_discr_delta_vector = []
    else:        
        obtained_effect = env.identify_effect_3d(initial_obj_pos,
                                                 current_obj_pos)
        if expected_effect == obtained_effect:
                res = 'success'
        else:
                res = 'false_pos'
                
        for tmp_delta in delta_vector:
            traj_inferred_discr_delta_vector.append(
                [obtained_effect, 
                 tmp_delta[0], 
                 tmp_delta[2],
                 tmp_delta[1]])
    
    if sim_param.debug_infer:
        print(expected_effect, obtained_effect)
        print('Result :', res.upper())
    
    return res, delta_nb_var_res, eef_traj, \
            delta_class_vector, \
            obtained_effect, \
            traj_inferred_discr_delta_vector


''' 
Compute score for each algorithm of a dataset type
Score = mean value of the perfomance value of an algorithm for a dataset
Example : for extended, hand-coded, mean perf_value when nb_init_pos = 4, 8, etc.
'''
def compute_score_norm(dataset_stat,
                  learn_algo_vector):
    dataset_score = 0
    for current_algo in learn_algo_vector:
        added_norm_perf = 0
        current_algo_class = dataset_stat.get_algo_results(current_algo)
        current_traj_dict = current_algo_class.get_results_dict()        
        for nb_init_pos, dataset_size_res_class in current_traj_dict.items():
            nb_trajs = sum(dataset_size_res_class.get_inference_res())
            max_performance = sim_param.perf_success_value*nb_trajs
            added_norm_perf += \
                (float(dataset_size_res_class.get_global_performance_value())/max_performance)*100
        algo_score = (added_norm_perf/len(current_traj_dict.keys()))
        current_algo_class.set_algo_score(algo_score)
        dataset_score += algo_score
    
    dataset_stat.set_dataset_score(float(dataset_score)/len(learn_algo_vector))

'''
Check if the posterior score is better than the previous one
'''
def check_better_score_norm(curr_score_vector, 
                       prev_score_vector):
            
    ## global score higher 
    if curr_score_vector[0] == prev_score_vector[0]:
        return 'equal'
    elif curr_score_vector[0] > prev_score_vector[0]:
        return 'bigger'
    else:
        return 'smaller'


'''
Compute cumulated performance value of each init position
given a dataset and a dicretization configuration
Store vector representing for each dataset, for each (algo, nb_init_pos) 
each position of the vector the perf_value of a init_pos, in range (0,nb_init_pos)
'''
def compute_perf_value_for_init_pos(current_dataset,
                                   current_algo,
                                   nb_init_pos_vector):

    max_added_perf_value = sim_param.perf_success_value * \
                            len(sim_param.effect_values)

    dataset_cumulated_perf_value_dict = OrderedDict()
    current_infer_algo_class = current_dataset.get_algo_results(current_algo)
    current_infer_algo_dict = current_infer_algo_class.get_results_dict()
    for current_nb_init_pos in nb_init_pos_vector: ## 8, 16, 32
        if sim_param.debug:
            print("current_dataset", current_dataset.get_dataset_name())
            print("current_nb_init_pos", current_nb_init_pos)
        current_dataset_size_class = \
            current_infer_algo_dict[current_nb_init_pos]
        current_dataset_size_dict = \
            current_dataset_size_class.get_indiv_traj_info()
        cumulated_perf_value_vector = []
        for current_init_pos in range(current_nb_init_pos):
            if sim_param.debug:
                print("current_init_pos", current_init_pos)
            added_perf_value = 0
            for current_effect in sim_param.effect_values:
                current_pos_effect_perf = \
                    current_dataset_size_dict[(current_init_pos, 
                                         current_effect)].get_perf_value()
                added_perf_value += current_pos_effect_perf
            added_perf_value = (float(added_perf_value)/max_added_perf_value)*100            
            if sim_param.debug:                
                print("Cumulated perf value", added_perf_value)
            cumulated_perf_value_vector.append(added_perf_value)
            
        dataset_cumulated_perf_value_dict[current_nb_init_pos] = \
            cumulated_perf_value_vector
            
    return dataset_cumulated_perf_value_dict
    
#'''
#Plot new inferred traj
#'''
#def plot_save_infere_traj(filepath,
#                     eef_pos, effect, delta_nb_var, 
#                     eef_traj,
#                     current_obj_pos,
#                     res,
#                     nb_initial_pos,
#                     current_orien,current_dist):                            
#    
#    fig = plt.gcf()
#    fig.set_size_inches(8, 9)
#    gs = gridspec.GridSpec(2, 1, height_ratios=[5,0.15])
#    ax = plt.subplot(gs[0])
#    ax2 = plt.subplot(gs[1])
#    
#    # set axis limits
#    ax.set_xlim(-1.5,1.5)
#    ax.set_ylim(-1.5,1.5)
#     
#    # plot the origin
#    if res == 'success':
#        obj_color = 'lightgreen'
#    elif res == 'false_pos':
#        obj_color = 'lightgrey'
#    elif res == 'fail':                
#        obj_color = 'grey'
#    else:
#        print('ERROR - plot_save_infere_traj . wrong res value')
#        sys.exit()
#    
#    ## plot object
#    obj=Rectangle((current_obj_pos[0] - sim_param.obj_side/2, 
#                        current_obj_pos[1] - sim_param.obj_side/2), 
#                        sim_param.obj_side,sim_param. obj_side, 
#                        fc=obj_color)
#    ax.add_patch(obj)
#    
#    ## plot distance circles discretization
#    if sim_param.distance_param:
#        if sim_param.discr_hand_coded:
#            remote_far_boundary = Circle(current_obj_pos, 
#                                    radius=sim_param.remote_far_boundary_value,
#                                    ls='dashed', 
#                                    edgecolor="grey",
#                                    fill=False)
#            far_close_boundary = Circle(current_obj_pos, 
#                                    radius=sim_param.far_close_boundary_value,
#                                    ls='dashed',
#                                    edgecolor="grey",
#                                    fill=False)                    
#            ax.add_patch(remote_far_boundary)
#            ax.add_patch(far_close_boundary)                                                                                        
#        else:
#            dist_sections = current_dist.get_raw_sections()
#            for v in dist_sections [1:-1]:
#                tmp_boundary = Circle(current_obj_pos, 
#                                        radius = v,
#                                        ls='dashed', 
#                                        edgecolor="grey",
#                                        fill=False)
#                ax.add_patch(tmp_boundary)
#    
#    ## plot orientation discretization
#    if not sim_param.discr_hand_coded :
#        angle_sections = current_orien.get_raw_sections()
#        for current_angle in angle_sections:
#            ax.plot([sim_param.obj_pos[0], cos(current_angle)],
#                    [sim_param.obj_pos[1], sin(current_angle)],
#                     ':', c='grey')
#    else:
#        ## plot lines identifying the discretization
#        ax.plot([sim_param.obj_side/2, sim_param.obj_side/2],
#                [1,-1],
#                ':', c='grey')
#        ax.plot([-sim_param.obj_side/2, -sim_param.obj_side/2],
#                [1,-1],
#                ':', c='grey')
#        ax.plot([1,-1],
#                [sim_param.obj_side/2, sim_param.obj_side/2],
#                ':', c='grey')
#        ax.plot([1,-1],
#                [-sim_param.obj_side/2, -sim_param.obj_side/2],
#                ':', c='grey')                                
#        
#    
#    ## plot the big circle points         
#    list_x_axis, list_y_axis, list_z_axis = \
#        setup.gen_init_eef(nb_initial_pos)
#    ax.plot(list_x_axis,list_y_axis,'.',c='grey')   
#    
#    ## identify current initial point
#    ax.plot(eef_traj[0][0],eef_traj[0][1],'.',markersize=20,c='darkgrey')
#    
#    ## print effect in the right left corner
#    ax.text(-1.25, 1.25, effect.upper() + " - " + res, fontsize=14)
#    
#    ## plot each segment of the new traj
#    tmp = len(sim_param.colormap_values)
#    nb_bins = float(1)/tmp
#    
#    for i in range(len(eef_traj)-1):            
#    
#        ## choose color based on probability
#        prob_value_segment = eef_traj[i+1][2]
#        if prob_value_segment <= nb_bins:
#            color = sim_param.colormap_values[0]
#        elif prob_value_segment <= nb_bins*2:
#            color = sim_param.colormap_values[1]
#        elif prob_value_segment <= nb_bins*3:
#            color = sim_param.colormap_values[2]
#        else:
#            color = sim_param.colormap_values[3]
#        
#        ## plot the segment
#        ax.plot([eef_traj[i][0],eef_traj[i+1][0]],
#                [eef_traj[i][1],eef_traj[i+1][1]],
#                '-*', c=color,
#                linewidth=2)
##        print(effect + '_' + strftime("%Y-%m-%d_%H:%M:%S", gmtime()) + str(i))
##        plt.savefig('/home/maestre/Desktop/tmp_plots/'+ \
##                    strftime("%Y-%m-%d_%H:%M:%S", gmtime()) + \
##                    str(i)+'.png')
#
#    ## draw a colorbar
#    colormap_bins = np.arange(0,1.1,nb_bins)
#    my_cmap_dist = colors.ListedColormap(sim_param.colormap_values)
#    my_norm_dist = colors.BoundaryNorm(colormap_bins, my_cmap_dist.N)
#    colorbar.ColorbarBase(ax2, cmap=my_cmap_dist,
#                                    norm=my_norm_dist,
#                                    boundaries=colormap_bins,
#                                    ticks=colormap_bins,
#                                    orientation='horizontal')
#
#    if sim_param.save_trajs or sim_param.save_some_trajs:
#        plt.savefig(filepath) 
#
#    if sim_param.plot_trajs or sim_param.plot_some_trajs:
#        plt.show()
            
'''
Find pos of value given label 
Output: pos
'''
def findPosLabel(rand_var,label):
    label_found = False
    pos = 0
    while not label_found and pos < rand_var.domainSize():
        if rand_var.label(pos) == label:
            label_found = True
            return pos
        pos += 1

    if not label_found:
        return -1
        
'''
Infere next movement

bn = current bn
ie = inference setup
discr_values = 

'''
def infere_mov(bn, ie, node_names, node_values):    

    ''' Add soft evidence '''        
    ## generate dict with current node values to apply
    values_dict = dict()
    for i in range(len(node_names)): 
        current_node = node_names[i]
        current_value = node_values[i]
        values_dict[current_node] = current_value

    if sim_param.debug:
        for i in values_dict:
            print (i, values_dict[i])

    ## transform each node values for the correspondant array 
    ## with 1 in the position of the current node value
    ## ej : if the possible node values are a b c, and we want to set c
    ## then it is 0 0 1
    for key in values_dict:        
        random_var = bn.variable(bn.idFromName(key))
        pos = findPosLabel(random_var,values_dict[key])
        if pos==-1:
            print("Label not found for",str(key),values_dict[key])
        else:
            tmp=np.zeros(random_var.domainSize())
            for p in range(len(tmp)):
                if p==pos:
                    tmp[p] = 1
                else:
                    tmp[p] = 0
            values_dict[key] = tmp.tolist()
        
    ie.setEvidence(values_dict)
    
    ''' Infere next movement to be done '''    
    ie.makeInference()
    
    ## get posterior    
    posterior = ie.posterior(bn.idFromName('move'))
#    if sim_param.debug:
    print(posterior)
#    print(posterior[0])
#    print(posterior[-1])
#    print(type(posterior))
#    print(type(posterior[0]))
#    print(posterior.variablesSequence()[0])
#    print(type(posterior.variablesSequence()[0]))
#    print(posterior.variablesSequence()[0].labels())
    

#    ## check if all values of 'move' have been generated
    move_var = bn.variable(bn.idFromName('move'))    
    move_nb_var = move_var.domainSize()
#    if move_nb_var != len(sim_param.move_values):
#        print('ERROR - infere_mov- Some move value is not available in the dataset')
#        sys.exit()

    ## get used moves
    move_list = []
    for pos in range(move_nb_var):
        move_list.append((posterior.variablesSequence())[0].label(pos))
        pos += 1

    ## select value of next move
    ## check if some prob is very high, or if 2 variables have a similar value
    posterior_list = []
#    for i in range(len(sim_param.move_values)):
    for i in range(len(move_list)):
        posterior_list.append(posterior[i])
        
    if all(x==posterior_list[0] for x in posterior_list):## no prob inferred
#        posterior_pos = 0
        posterior_pos = random.randint(0,len(posterior_list)-1)
        posterior_value = posterior_list[posterior_pos]
    else:
        max_value = max(posterior_list)
        posterior_pos = posterior_list.index(max_value)
        posterior_value = posterior_list[posterior_pos]
#        max_value = max(posterior_list)
#        max_pos = posterior_list.index(max_value)
#        
#        if max_value > 0.9: ## very high
#            posterior_pos = max_pos              
#        else: ## several probs
#            posterior_list[max_pos] = 0
#            second_max_value = max(posterior_list)
#            second_max_pos = posterior_list.index(max(posterior_list))
#
##            if max_value > 2*second_max_value: ## far probs
##                posterior_pos = max_pos
##            else: ## close probs
##                posterior_pos = random.choice([max_pos, second_max_pos])
#
#            if max_value == second_max_value:
#                posterior_pos = random.choice([max_pos, second_max_pos])
#            else:
#                posterior_pos = max_pos
#            
#        if posterior_pos == max_pos:
#            posterior_value = max_value
#        else:
#            posterior_value = second_max_value
            
    move_value = (posterior.variablesSequence())[0].label(posterior_pos)    
    if sim_param.debug:
        print("Best posterior of move is", move_value.upper(),\
            "with prob", posterior_value, "in position", posterior_pos, '\n')      
    
    return move_value, round(posterior_value,3), int(move_nb_var)
            
