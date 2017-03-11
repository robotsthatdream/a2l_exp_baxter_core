#!/usr/bin/env python

# -*- coding: utf-8 -*-
"""
@author: maestre
"""
from __future__ import print_function

from time import gmtime, strftime
from collections import OrderedDict
import numpy as np

import os, sys
lib_a2l_path = os.path.realpath(os.path.abspath(os.path.join('.', 'lib', 'a2l_core_lib')))
sys.path.append(lib_a2l_path)
lib_exp_path = os.path.realpath(os.path.abspath(os.path.join('.', 'lib', 'experiment_lib')))
sys.path.append(lib_exp_path)
import dataset_generation as dataset
import dataset_stats as dataset_stats
import statistics as stats
import simulation_parameters as sim_param
import experiment_a2l as exp_a2l
import experiment_discretization as exp_discr
import inference_ros as infer_ros
import iteration_classes as iter_class
import ros_services

'''
Print current experiment parameters
'''
def print_sim_param(print_values, 
                    current_orien_discr,
                    current_inclin_discr,
                    current_dist_discr):    

    if sim_param.experiment_type == 'a2l_dataset_extension' or \
       sim_param.experiment_type == 'a2l_reproduce_dataset':
        print('\n-----------------EXPERIMENT - LEARN WITH A2L, and EXTEND DATASET')
        print('\nINIT POSITIONS USED :', print_values[0])
    elif sim_param.experiment_type == 'discretization_selection':
        print('\n-----------------EXPERIMENT - SELECT DISCRETIZATION CONFIGURATION')
        print('\nORIENTATION DISCR VALUES :', print_values[0])
        print('DISTANCE DISCR VALUES :', print_values[1])
    else:
        print('ERROR - print_sim_param - exp type unknown')
        sys.exit()    

    current_score = 'BDEU'
    if sim_param.score_bic:
        current_score = 'BIC'
    elif sim_param.score_aic:
        current_score = 'AIC'
    elif sim_param.score_likelihood:
        current_score = 'LIKELIHOOD'
    print('\nSCORE :', current_score)

    variables = ['effect', 'orientation', 'move']
    if sim_param.distance_param:
        variables += ['distance']
    print('\nVARIABLES :', variables)

    print('\nFIXED OBJ POSITION :', sim_param.fixed_obj_pos)
    
    print('\nMIN NB INIT_POS :', sim_param.nb_min_init_pos)
    
    print('\nPREDEFINED DISCRETIZATION:', sim_param.discr_hand_coded)
    
    print('\nTRAJECTORY EXTENSION VALUE', sim_param.extend_max_trajs)
    
    if not sim_param.discr_hand_coded:
        print('ORIENTATION SECTIONS :', sim_param.nb_min_orientation_sections)
        print('INCLINATION SECTIONS :', sim_param.nb_min_inclination_sections)
        print('DISTANCE SECTIONS :', sim_param.nb_min_distance_sections)
        print('DISCRETIZED INITIAL RANDOM MOVS:', sim_param.semi_random_trajs)
        
        if sim_param.discr_random:                    
            print()
            angle_format = 'degrees'
            current_orien_discr.print_me(angle_format)
            print()
            angle_format = 'degrees'
            current_inclin_discr.print_me(angle_format)            
            print()
            current_dist_discr.print_me()
            

'''
Generate folders
'''
def generate_folders():
    current_generated_files_folder = os.getcwd() + '/' + sim_param.generated_files_folder
    if not os.path.exists(current_generated_files_folder):
        os.makedirs(current_generated_files_folder)    
    
    if sim_param.save_trajs or sim_param.save_some_trajs:
        if sim_param.experiment_type == 'change_dataset_size':
            exp_folder = sim_param.exp_dataset_size_folder
            extra_info = '_' + str(sim_param.nb_min_orientation_sections) + \
                         '_' + str(sim_param.nb_min_distance_sections)
        elif sim_param.experiment_type == 'change_discretization':
            exp_folder = sim_param.exp_discr_folder
            extra_info = '_' + str(sim_param.nb_min_init_pos)
        else:
            print('ERROR - generate_folders - exp type unknown')
            sys.exit()
            
        current_results_folder = os.getcwd() + '/' + \
                        sim_param.results_folder + \
                        exp_folder + \
                        strftime("%Y-%m-%d_%H:%M:%S", 
                        gmtime()) + \
                        extra_info + \
                        '/'
                            
        if not os.path.exists(current_results_folder):
            os.makedirs(current_results_folder)
    else:
        current_results_folder = ''
    
    current_plot_folder = os.getcwd() + '/' + sim_param.plots_folder
    if not os.path.exists(current_plot_folder):
        os.makedirs(current_plot_folder)    

    return current_generated_files_folder, current_results_folder, current_plot_folder
    
    
'''
Main
'''
if __name__ == "__main__":
 
    ''' Experiment configuration '''
#    dataset_type_vector = ['directed']
    dataset_type_vector = ['random']    
#    dataset_type_vector = ['directed', 'random']

#    learn_algo_vector = ['hard-coded', 'hillclimbing', 'k2']    
#    learn_algo_vector = ['hillclimbing']
    learn_algo_vector = ['hard-coded']
#    learn_algo_vector = ['hard-coded', 'hillclimbing']    

    ''' Create folders '''
    current_generated_files_folder, current_results_folder, current_plot_folder = \
        generate_folders()

    ''' Update number of initial positions in ROS '''
    ros_services.update_nb_init_pos()
    
    ''' Restart scenario if box is not in init position '''
    obj_pos_dict = OrderedDict()
    for obj_name in sim_param.obj_name_vector:
        obj_pos = ros_services.call_get_model_state(obj_name)
        obj_pos = [round(pos, sim_param.round_value+1) for pos in obj_pos[0:3]]
        obj_pos_dict[obj_name] = obj_pos 
        
        cube_pos = obj_pos_dict['cube']
        if cube_pos[0] != sim_param.first_obj_pos[0] or \
                          sim_param.first_obj_pos[1] or \
                          sim_param.first_obj_pos[2]:
            success = ros_services.call_restart_world("all")
            if not success:
                print("ERROR - restart_world failed")    
    
    ''' Select obj position during inference ''' 
    model_state = ros_services.call_get_model_state(sim_param.obj_name_vector[0])
    initial_obj_pos = model_state[0:3]
    
    '''  Run experiment to select the dicretization configuration'''    
    if sim_param.experiment_type == 'discretization_selection':
        dataset_stats_vector, \
        orientation_discr_vector, \
        distance_discr_vector = exp_discr.run_exp_dataset_size(
                                    dataset_type_vector,
                                    learn_algo_vector,
                                    initial_obj_pos,
                                    current_generated_files_folder, 
                                    current_results_folder, 
                                    current_plot_folder)
    
        ''' Plot stats '''
        for dataset_stat in dataset_stats_vector:        
            stats.plot_performance_changing_discretization(
                dataset_stat,
                current_plot_folder,
                orientation_discr_vector,
                distance_discr_vector,
                learn_algo_vector)
                                                    
        ''' Print experiment main configuration '''
        print_sim_param([orientation_discr_vector,
                         distance_discr_vector])

        ''' Run experiment A2L'''        
    elif sim_param.experiment_type == 'a2l_dataset_extension' or \
         sim_param.experiment_type == 'a2l_reproduce_dataset':
        
        print('\n\n----------------------------------------------------------')
        print('----------------------------------------------------------')
        print('----------------------------------------------------------')
        print('----------------------------------------------------------')
        print('----------------------------------------------------------')
        print('BASIC LEARNING')
        print('----------------------------------------------------------')
        print('----------------------------------------------------------')
        print('----------------------------------------------------------')
        print('----------------------------------------------------------')
        print('----------------------------------------------------------\n\n')        
        
        ''' 
        Run basic learning        
        '''
        dataset_stats_vector, initial_pos_vector, \
        current_orien_discr, current_inclin_discr, \
        current_dist_discr, \
        initial_raw_delta_vector, filename = \
            exp_a2l.basic_learning_dataset_size(
                dataset_type_vector,
                learn_algo_vector,
                current_results_folder)

        if sim_param.experiment_type == 'a2l_dataset_extension':
                                        
            ''' Compute score '''
            for dataset_stat in dataset_stats_vector:
                infer_ros.compute_score_norm(dataset_stat, learn_algo_vector)
    
            '''
            Compute cumulated performance value of each init position
            given a dataset and a discretization configuration
            '''        
            dataset_perf_value_vector = \
                dataset.compute_cumulated_perf(
                    dataset_stats_vector[0],
                    learn_algo_vector,
                    initial_pos_vector)
            
            
            ''' Plot stats basic learning'''
            if sim_param.plot_stats:            
                ## plot dataset generated
                dataset_stats.plot_dataset_stats(filename)
                
                if sim_param.nb_dataset_sizes == 1:
                    ## plot results of inferred trajectories for 1 size
                    stats.plot_global_results_dataset_run(current_plot_folder, 
                                                           dataset_stats_vector,
                                                           dataset_type_vector,
                                                           learn_algo_vector,
                                                           sim_param.nb_min_init_pos)
                else:            
                    for dataset_stat in dataset_stats_vector:
                        ## plot results of inferred trajectories for diff sizes
                        stats.plot_global_results_dataset_size(dataset_stat,
                                                               current_plot_folder,
                                                               learn_algo_vector)
                                                               
                        ## plot mean move's prob for diff sixes
                        stats.plot_probs_dataset_size(dataset_stat,                                                           
                                                      current_plot_folder,
                                                      learn_algo_vector)
                
    #            ## plot circles with number of reproduced effects
    #            ## with different dataset sizes
    #            for current_dataset in dataset_perf_value_vector:
    #                stats.plot_infer_trajs_performance_values_changing_size(
    #                        current_dataset,
    #                        initial_obj_pos,
    #                        initial_pos_vector,
    #                        learn_algo_vector)
                                                  

        if 'random' in dataset_type_vector:
            print('\n\n----------------------------------------------------------')
            print('----------------------------------------------------------')
            print('----------------------------------------------------------')
            print('----------------------------------------------------------')
            print('----------------------------------------------------------')
            print('ADAPTIVE LEARNING')
            print('----------------------------------------------------------')
            print('----------------------------------------------------------')
            print('----------------------------------------------------------')
            print('----------------------------------------------------------')
            print('----------------------------------------------------------\n\n')        

            ''' Compute some initial variables '''
            extended_initial_pos_vector = [sim_param.nb_init_pos_for_adaption]
            max_succ_value = sim_param.perf_success_value * \
                            len(sim_param.effect_values) * \
                            sim_param.nb_init_pos_for_adaption 

            ''' Adapt the result of basic learning and add it as iteration 0 '''
            ## get only random dataset stats from initial dataset vector
            for tmp_dataset_stats in dataset_stats_vector:
                dataset_name = tmp_dataset_stats.get_dataset_name()
                if dataset_name == 'random':
                    initial_random_dataset_stats = tmp_dataset_stats
                    
            for tmp_perf_dataset_stats in dataset_perf_value_vector:   
                dataset_name = tmp_perf_dataset_stats.get_dataset_name()
                if dataset_name == 'random':                 
                    initial_random_dataset_perf_value_stats = \
                        tmp_perf_dataset_stats
                                           
            ## Instead of using the current dataset score on the initial 
            ## dataset, computed for different sizes, 
            ## we get the normalized score for current size as dataset score
            ## Also for the algo score '''                
            previous_iter_raw_delta_vector = initial_raw_delta_vector
            tmp_algo_score_vector = []
            for current_algo in learn_algo_vector:                
                current_algo_class = initial_random_dataset_stats.get_algo_results(current_algo)
                current_algo_dict = current_algo_class.get_results_dict()                                   
                tmp_algo_size_perf_value = \
                    (current_algo_dict[sim_param.nb_init_pos_for_adaption]).get_global_performance_value()
                tmp_algo_size_score_norm = (float(tmp_algo_size_perf_value)/max_succ_value)*100
                tmp_algo_score_vector.append(tmp_algo_size_score_norm)
                ## update normalized score
                current_algo_class.set_algo_score(tmp_algo_size_score_norm)                
                                    
            previous_iter_score_vector = [sum(tmp_algo_score_vector)]
            previous_iter_score_vector = previous_iter_score_vector + \
                                         tmp_algo_score_vector
            ## update normalized score
            initial_random_dataset_stats.set_dataset_score(sum(tmp_algo_score_vector))
            
            ## to store the iterations data
            iterations_dict = OrderedDict()
            
            ## add the updated initial info as initial record
            initial_iter_data_class = iter_class.Iteration_data(
                 [initial_random_dataset_stats], ## as a vector only adding random class
                 [initial_random_dataset_perf_value_stats], ## as a vector only adding random class
                 initial_raw_delta_vector, 
                 True,
                 []) ## to always plot the first iteration
            iterations_dict[0] = initial_iter_data_class  

            previous_iter_dataset_stats_class = initial_random_dataset_stats
            
            ## store perf value change for each init pos and effect 
            ## between iterations (if change restarts to 0)
            perf_value_changes_vector = np.zeros(
                sim_param.nb_init_pos_for_adaption * len(sim_param.effect_values))
            prev_iter_perf_value_initpos_effect_vector = \
                ['fail' for i in range (sim_param.nb_init_pos_for_adaption * 
                len(sim_param.effect_values))]
            
            ''' Generate new iterations '''
            inferred_discr_delta_vector = [] ## delta knowledge created in the prev evaluation
            for nb_iter in range(1,sim_param.nb_adapted_iterations+1): ## 0 already available
                ## only random dataset in the vector
                current_iter_dataset_stats_class_stats_vector,\
                current_iter_raw_delta_vector, \
                effect_directed_extension_vector, \
                inferred_discr_delta_vector = \
                    exp_a2l.adaptive_learning_dataset_size(
                        previous_iter_dataset_stats_class,
                        sim_param.nb_init_pos_for_adaption, ## 8
                        learn_algo_vector,
#                        initial_obj_pos,
                        current_results_folder, 
                        current_orien_discr, 
                        current_inclin_discr,
                        current_dist_discr,
                        nb_iter,
                        previous_iter_raw_delta_vector,
                        perf_value_changes_vector,
                        inferred_discr_delta_vector)

                ''' Compute score '''
                current_iter_dataset_stats_class = current_iter_dataset_stats_class_stats_vector[0]
                infer_ros.compute_score_norm(current_iter_dataset_stats_class,
                                    learn_algo_vector)
                                        
                current_iter_score_vector = [current_iter_dataset_stats_class.get_dataset_score()]
                for current_algo in learn_algo_vector:                                        
                    current_algo_class = current_iter_dataset_stats_class.get_algo_results(current_algo)
                    current_algo_dict = current_algo_class.get_results_dict()                                   
                    tmp_algo_size_perf_value = \
                        (current_algo_dict[sim_param.nb_init_pos_for_adaption]).get_global_performance_value()
                    tmp_algo_size_score_norm = (float(tmp_algo_size_perf_value)/max_succ_value)*100
                    current_iter_score_vector.append(tmp_algo_size_score_norm)                    
                
                ''' Compute cumulated performance value for init_pos'''
                current_iter_dataset_stats_class_perf_value_vector = \
                    dataset.compute_cumulated_perf(
                        previous_iter_dataset_stats_class,
                        learn_algo_vector,
                        extended_initial_pos_vector)                    
                
                ''' Store iterations '''
                better_norm_score = infer_ros.check_better_score_norm(current_iter_score_vector, 
                                                        previous_iter_score_vector)
                current_iter_data_class = iter_class.Iteration_data(
                     current_iter_dataset_stats_class_stats_vector, ## only random
                     current_iter_dataset_stats_class_perf_value_vector, ## only random
                     current_iter_raw_delta_vector, ## only random
                     better_norm_score,
                     effect_directed_extension_vector) ## equal, better or smaller                     
                iterations_dict[nb_iter] = current_iter_data_class     

                ''' Compute cumulated performance value for init_pos and effect'''
                curr_iter_perf_value_initpos_effect_vector = \
                    dataset.plot_infer_trajs_iterations_effect_init_pos(
                        iterations_dict,
                        nb_iter,
                        current_algo)
                        
                ''' Update perf value changes count '''
                perf_value_changes_vector = \
                    dataset.update_perf_value_changes(
                        perf_value_changes_vector,
                        prev_iter_perf_value_initpos_effect_vector,
                        curr_iter_perf_value_initpos_effect_vector)
                prev_iter_perf_value_initpos_effect_vector = \
                    curr_iter_perf_value_initpos_effect_vector
                for i in range(0, len(perf_value_changes_vector), 4):
                    print('\n',
                            int(perf_value_changes_vector[i+0]), 
                            int(perf_value_changes_vector[i+1]),
                            int(perf_value_changes_vector[i+2]),
                            int(perf_value_changes_vector[i+3]))
                     
                ''' Update delta vector IF iteration improves the score '''
                print('\n--> Prev iter:', previous_iter_score_vector)
                print('\n--> Curr iter:', current_iter_score_vector)
                print('\n--> Equal or better score?', better_norm_score)
                if better_norm_score=='bigger' or better_norm_score=='equal': 
                    previous_iter_dataset_stats_class = current_iter_dataset_stats_class
                    previous_iter_raw_delta_vector = current_iter_raw_delta_vector
                    previous_iter_score_vector = current_iter_score_vector
                    if better_norm_score=='bigger':
#                        ## plot dataset generated
#                        filename = \
#                            sim_param.current_generated_files_folder + \
#                            'random_discr_wps_iteration_' + str(nb_iter) + '.csv'
#                        dataset_stats.plot_dataset_stats(filename)
                    
                        ## Plot result of effects for each init_pos in each iteration
                        for current_algo in learn_algo_vector:
                            stats.plot_infer_trajs_iterations_effect(
                                iterations_dict,
                                sim_param.nb_init_pos_for_adaption,
                                current_algo,
                                nb_iter+1)
                else :
                    print('\nTemporal new raw entries', 
                          len(current_iter_raw_delta_vector) - 
                          len(previous_iter_raw_delta_vector))
                
                print("\nRaw dataset size after iteration", nb_iter,
                      'is', len(previous_iter_raw_delta_vector))
                    
            ''' Plot stats for new dataset '''
            if sim_param.plot_stats:        
                for current_dataset_stat in current_iter_dataset_stats_class_perf_value_vector:
                    stats.plot_infer_trajs_performance_values_changing_size(
                            current_dataset_stat,
                            initial_obj_pos,
                            extended_initial_pos_vector,
                            learn_algo_vector)
                     
            ''' Plot stats to compare iterations'''            
            stats.plot_infer_trajs_iterations_performance_value(
                iterations_dict,
                sim_param.nb_init_pos_for_adaption,
                learn_algo_vector)

            stats.plot_infer_trajs_iterations_score(
                iterations_dict,
                current_plot_folder,
                learn_algo_vector)
                
            ## Plot result of effects for each init_pos in each iteration
            for current_algo in learn_algo_vector:
                stats.plot_infer_trajs_iterations_effect(
                    iterations_dict,
                    sim_param.nb_init_pos_for_adaption,
                    current_algo,
                    nb_iter)
                    
            ## plot last dataset generated
            filename = \
                sim_param.generated_files_folder + \
                'random_discr_wps_iteration_' + str(nb_iter) + '.csv'
            dataset_stats.plot_dataset_stats(filename)
                    
        ''' Print experiment main configuration '''
        print_sim_param([initial_pos_vector], 
                        current_orien_discr, 
                        current_inclin_discr,
                        current_dist_discr)    
        
    else:
        print('ERROR - main - exp type unknown')
        sys.exit()                                             
         
    print('\nDone.')