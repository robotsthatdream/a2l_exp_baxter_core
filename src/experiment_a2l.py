# -*- coding: utf-8 -*-
"""
@author: maestre
"""
from __future__ import print_function

import os, sys
lib_a2l_path = os.path.abspath(os.path.join('.', 'lib', 'a2l_core_lib'))
sys.path.append(lib_a2l_path)
lib_exp_path = os.path.abspath(os.path.join('.', 'lib', 'experiment_lib'))
sys.path.append(lib_exp_path)
import dataset_generation as dataset
import discretize as discr
import inference_ros as infer_ros
#import inf_traj_single_dataset_classes as dataset_single_classes
import learning as ln
import simulation_parameters as sim_param
import inf_traj_series_dataset_classes as dataset_series_classes
if sim_param.print_time:
        import time        
import ros_services

'''
Run basic learning
'''
def basic_learning_dataset_size(dataset_type_vector,
                                learn_algo_vector,
#                                current_obj_pos,
                                current_results_folder):
          
    ## Create discretizations
    current_orien_discr = discr.compute_orientation_discr()
    current_inclin_discr = discr.compute_inclination_discr()
    current_dist_discr = discr.compute_distance_discr()
    
    dataset_stats_vector = [] 
    for dataset_type in dataset_type_vector:
        print('\n\n----------------------------------------------------------')
        print('----------------------------------------------------------')
        print('Generating and discretizing', dataset_type.upper(), 'dataset')
        print('----------------------------------------------------------')
        print('----------------------------------------------------------\n\n')        
        if dataset_type == 'directed':
            filename = sim_param.generated_files_folder + 'directed_discr_wps.csv'
        elif dataset_type == 'random':
            filename = sim_param.generated_files_folder + 'random_discr_wps.csv'        
        else:
            print('ERROR - main - wrong dataset_type value')
   
        ## Create the classes to store the different stats results 
        ## Dataset            
        current_dataset_stats = dataset_series_classes.Dataset_stats(dataset_type)
        ## Algo
        for tmp_algo in learn_algo_vector:
            tmp_algo_stats = dataset_series_classes.Algo_stats(tmp_algo)
            current_dataset_stats.add_algo_results(tmp_algo, 
                                                   tmp_algo_stats)
                                                   
        basic_size = sim_param.nb_min_init_pos       
        nb_initial_pos_vector = [basic_size * repetitions 
                                for repetitions in 
                                range(1, sim_param.nb_dataset_sizes+1)]            
        for current_nb_initial_pos in nb_initial_pos_vector: # 4, 8, 12, 16
            print('\n\n----------------------------------------------------------')
            print('----------------------------------------------------------')
            print('# ARM INITIAL POSITIONS =', str(current_nb_initial_pos))
            print('----------------------------------------------------------')
            print('----------------------------------------------------------\n\n')                         
        
            ## create raw dataset
            if sim_param.new_dataset:
                dataset_filename = \
                    ros_services.call_generate_directed_dataset(dataset_type)
            else:
                dataset_filename = \
                    '/home/maestre/indigo/baxter_ws/src/a2l_exp_baxter_actions/src/generated_datasets/directed_dataset.csv'        
            
            ## read dataset            
            raw_delta_vector = dataset.read_dataset(dataset_filename)
            
            ## discretize dataset
            discr_delta_vector = discr.discretize_trajs(
                                        raw_delta_vector,
                                        current_orien_discr,
                                        current_inclin_discr,
                                        current_dist_discr,)
                                    
            if dataset_type == 'random': ## remove the very small deltas
                nb_removed = 0
                for discr_delta in discr_delta_vector:
                    if discr_delta[2] == 'zero':
                        discr_delta_vector.remove(discr_delta)
                        nb_removed += 1
                if nb_removed > 0:
                    print(nb_removed,'very small movements were removed')                        
            
            ## store discretized dataset
            discr.save_discr_deltas(filename, discr_delta_vector)
            
            ## plot hard-coded discretized dataset stats
            if sim_param.plot_dataset_stats and sim_param.discr_hand_coded:
                discr.plot_dataset_stats(filename)
                
            for current_algo in learn_algo_vector:            
                print('\n\n----------------------------------------------------------')
                print('----------------------------------------------------------')
                print('Learning BN in the', dataset_type.upper(), 'dataset',
                      'with', current_algo.upper(),  'algorithm')
                print('----------------------------------------------------------')
                print('----------------------------------------------------------\n\n')                   

                dataset_size_class = \
                    dataset_series_classes.Dataset_size_results(current_nb_initial_pos)
                                                       
                ## Learn BN
                if sim_param.print_time:
                    t = time.process_time()
                bn = ln.learn_bn(filename, current_algo)
                bn_url = 'BN_' + dataset_type + '_' + current_algo + '.bif'                
                ln.save_bn(bn, 
                           sim_param.generated_files_folder + bn_url)                           
                if sim_param.print_time:
                    print('Elapsed time for learn_bn', 
                          time.process_time() - t)
                                
                infer_ros.infere_trajectories(
                    current_results_folder,
                    bn, 
                    current_algo,
                    dataset_size_class,
                    dataset_type,
#                    current_obj_pos,                    
                    current_nb_initial_pos,
                    0, ## number of traj
                    current_orien_discr,
                    current_inclin_discr,
                    current_dist_discr)
                                               
                ## Store stats and move's mean prob                 
                current_algo_stats_class = \
                    current_dataset_stats.get_algo_results(current_algo)                
                current_algo_stats_class.add_result(current_nb_initial_pos,
                                                    dataset_size_class)
                                                   
#                if sim_param.debug:
#                current_algo_stats_class.print_me()                                        
        dataset_stats_vector.append(current_dataset_stats)

    return dataset_stats_vector, \
            nb_initial_pos_vector, \
            current_orien_discr,\
            current_inclin_discr,\
            current_dist_discr, \
            raw_delta_vector, \
            filename
'''
Run adaptive learning
'''
def adaptive_learning_dataset_size(current_dataset_stats, ## random
                                    current_nb_initial_pos, ## 8
                                    learn_algo_vector,
#                                    current_obj_pos,
                                    current_results_folder, 
                                    current_orien_discr,
                                    current_inclin_discr,
                                    current_dist_discr,
                                    current_iteration,
                                    previous_raw_delta_vector,
                                    perf_value_changes_vector,
                                    prev_inferred_discr_delta_vector):
    
    ''' Extend dataset '''    
    dataset_stats_vector = [] 
    current_dataset_name = current_dataset_stats.get_dataset_name()
    ## Create the classes to store the different stats results 
    ## Dataset            
    extended_dataset_stats = dataset_series_classes.Dataset_stats('random')
    ## Algo
    for tmp_algo in learn_algo_vector:
        tmp_algo_stats = dataset_series_classes.Algo_stats(tmp_algo)
        extended_dataset_stats.add_algo_results(tmp_algo, 
                                               tmp_algo_stats)

    print('\n\n----------------------------------------------------------')
    print('----------------------------------------------------------')
    print('Extending', current_dataset_name.upper(), 'dataset')
    print('ITERATION', current_iteration)    
    print('----------------------------------------------------------')
    print('----------------------------------------------------------\n\n')   
    
    ## For each successful trajectory generate new ones
    new_raw_delta_vector, effect_directed_extension_vector = \
        dataset.extend_dataset(
            current_dataset_stats,
            sim_param.nb_init_pos_for_adaption,
            learn_algo_vector,
            perf_value_changes_vector,
            len(previous_raw_delta_vector))
    
    extended_raw_delta_vector = previous_raw_delta_vector + \
                                new_raw_delta_vector    
    
    ## discretize dataset
    discr_delta_vector = discr.discretize_trajs(
                                extended_raw_delta_vector,
                                current_orien_discr,
                                current_inclin_discr,
                                current_dist_discr)
    discr_delta_vector = discr_delta_vector + prev_inferred_discr_delta_vector
    
    ## remove small moves
    nb_removed = 0
    for discr_delta in discr_delta_vector:
        if discr_delta[3] == 'zero':
            discr_delta_vector.remove(discr_delta)
            nb_removed += 1
    if nb_removed > 0:
        print(nb_removed,'very small movements were removed')                        
    
    ## store discretized dataset
    filename = sim_param.generated_files_folder + \
                'random_discr_wps_iteration_' + str(current_iteration) + '.csv'
    discr.save_discr_deltas(filename, discr_delta_vector)                
    
    inferred_discr_delta_vector = []
    for current_algo in learn_algo_vector:            
        print('\n\n----------------------------------------------------------')
        print('----------------------------------------------------------')
        print('Learning BN in the EXTENDED', current_dataset_name.upper(), 'dataset',
              'with', current_algo.upper(),  'algorithm')
        print('----------------------------------------------------------')
        print('----------------------------------------------------------\n\n')                   
                                               
        ## Learn BN
        bn = ln.learn_bn(filename, current_algo)
        bn_url = 'BN_' + current_dataset_name + '_' + current_algo + '.bif'                
        ln.save_bn(bn, 
                   sim_param.generated_files_folder + bn_url)                           
     
        ''' Evaluation '''
        dataset_size_class = \
            dataset_series_classes.Dataset_size_results(current_nb_initial_pos)        
        tmp_inferred_discr_delta_vector = \
            infer_ros.infere_trajectories(
                current_results_folder,
                bn, 
                current_algo,
                dataset_size_class,
                current_dataset_name,
#                current_obj_pos,            
                current_nb_initial_pos,
                current_iteration,
                current_orien_discr,
                current_inclin_discr,
                current_dist_discr)
        if len(tmp_inferred_discr_delta_vector) > 0:
            print(current_algo, len(tmp_inferred_discr_delta_vector))
            inferred_discr_delta_vector = inferred_discr_delta_vector +\
                                          tmp_inferred_discr_delta_vector        
                                                                         
        ## Store stats and move's mean prob
        extended_algo_stats_class = \
            extended_dataset_stats.get_algo_results(current_algo)                
        extended_algo_stats_class.add_result(current_nb_initial_pos,
                                            dataset_size_class)
                                           
        if sim_param.debug:
            extended_algo_stats_class.print_me()
    dataset_stats_vector.append(extended_dataset_stats)
            
    ''' Return stats, extended dataset and vector of extended (initpos, effect)'''
    return dataset_stats_vector, \
            extended_raw_delta_vector, \
            effect_directed_extension_vector, \
            inferred_discr_delta_vector

            