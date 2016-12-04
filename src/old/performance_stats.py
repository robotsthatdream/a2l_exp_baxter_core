# -*- coding: utf-8 -*-
"""
Created on Wed Jul 13 10:51:01 2016

@author: maestre
"""

from __future__ import print_function

import matplotlib.pyplot as plt
import matplotlib as mpl
import random
import numpy as np

import os, sys
lib_path = os.path.abspath(os.path.join('..', 'lib'))
sys.path.append(lib_path)

import statistics_classes as stats_classes

import os, sys
run_path = os.path.abspath(os.path.join('..'))
sys.path.append(run_path)
import simulation_parameters as sim_param


'''
Plot performance values for different discretizations
'''
def plot_global_results_dataset_performance(dataset_stat,
                                            plot_folder,
                                            orien_vector,
                                            dist_vector,
                                            algo_vector):    
    ''' Transform input into matrix '''
    matrix_vector = []
    for current_algo in algo_vector:
        current_algo_class = dataset_stat.get_algo_results(current_algo)
        current_algo_stats = current_algo_class.get_results() 
        matrix = np.zeros(shape=(len(orien_vector),
                                    len(dist_vector)))
        matrix_norm = np.zeros(shape=(len(orien_vector),
                                    len(dist_vector)))                                    
        for pos_orien in range(len(orien_vector)):
            for pos_dist in range(len(dist_vector)):
                tmp_pos_orien = orien_vector[pos_orien]
                tmp_pos_dist = dist_vector[pos_dist]
                current_traj_class = current_algo_stats[(tmp_pos_orien,
                                                            tmp_pos_dist)]
                nb_trajs = sum(current_traj_class.get_traj_res())
                max_perf = 4*nb_trajs
                current_performance = current_traj_class.get_performance_value()
                matrix[pos_orien, pos_dist] = current_performance
                matrix_norm[pos_orien, pos_dist] = (float(current_performance)/max_perf)*100


#        print ('\n', matrix)                    
#        print ('\n', matrix_norm)
        matrix_vector.append((current_algo, matrix_norm))
    
    ''' Plot matrix '''    
    fig, (ax1, ax2, ax3) = plt.subplots(nrows=1, ncols=3, figsize=(12,2.5))
    
    ## labels
    fig.text(0.75, 1.1, dataset_stat.get_dataset_name().upper() + ' DATASET', 
             ha='right', va='center', style='italic')
    fig.text(0.43, -0.05, 'Distance discretization', 
             ha='center', va='center', fontsize=13)
    fig.text(0.08, 0.5, 'Orientation discretization', 
             ha='center', va='center', rotation='vertical', fontsize=13)
    plt.rcParams.update({'font.size': 11})    
    
    for current_algo, matrix_norm in matrix_vector:
    
        ## define several parameters and plot
        if current_algo == 'hand-coded':
            _tmp = 'Hand-coded'
            _ax = ax1
            
        elif current_algo == 'hillclimbing':
            _tmp = 'Hillclimbing'
            _ax = ax2
            
        elif current_algo == 'k2':
            _tmp = 'K2'
            _ax = ax3
            
        else:
            _tmp = ''  
                
        _ax.set_title(_tmp.upper()) 
        _ax.tick_params(axis='both', which='major', labelsize=12)
        _ax.tick_params(axis='both', which='minor', labelsize=12)
            
        ## plot matrix
        im = _ax.imshow(matrix_norm,
                   extent=[0,dist_vector[-1],0,orien_vector[-1]],
                   interpolation='nearest',
                   aspect='auto',
                   cmap='Blues',
                   vmin=0, vmax=100)
                   
        ## ticks
        dist_x = (dist_vector[1] - dist_vector[0])/2
        _ax.set_xticks(np.asarray(dist_vector) - dist_x)
        tmp = np.linspace(orien_vector[-1]/len(orien_vector), 
                          orien_vector[-1], 
                          len(orien_vector))
        dist_y = (tmp[1] - tmp[0])/2
        _ax.set_yticks(tmp - dist_y, minor=False)
        _ax.set_xticklabels(dist_vector, minor=False)
        _ax.set_yticklabels(orien_vector, minor=False)
    
    ## colorbar
    cax, kw = mpl.colorbar.make_axes([ax for ax in [ax1, ax2, ax3]], location='right')
    cbar = plt.colorbar(im, cax=cax, **kw)
    cbar.ax.set_yticklabels(np.linspace(0, 100, 11, endpoint=True))
    
    plt.show()

#    if sim_param.save_stats:         
#        if sim_param.discr_hand_coded:
#            discr_type = 'discr_provided'
#        else:
#            discr_type = 'discr_' + str(sim_param.nb_orientation_sections) + \
#                         '_' + str(sim_param.nb_distance_sections)
#        fig.savefig(plot_folder + 
#                    'stats_' +                     
#                    dataset_name +
#                    '_' + 
#                    discr_type,
#                    dpi = sim_param.dpi)
#                    
#    if sim_param.plot_stats:
#        plt.show()        
    

'''
Main
'''

## Fake stats
extended_stats = stats_classes.Dataset_stats('extended')
algo_vector = ['hand-coded', 'hillclimbing', 'k2']
orien_vector = [4, 8, 16, 32]
dist_vector = [2, 4, 6]

for current_algo in algo_vector:
    current_algo_stats = stats_classes.Algo_stats(current_algo)

    for current_dist in dist_vector:
        for current_orien in orien_vector:
    
            success_value = random.randint(0, 5)
            f_p_value = random.randint(0, 5)
            if success_value + f_p_value > 10:
                f_p_value = 10 - success_value
                if not f_p_value == 0:
                    f_p_value -= 1
                fail_value = 1
            else:
                fail_value = 10- success_value - f_p_value       
            res_stats = stats_classes.Trajectory_stats(
                            [success_value, f_p_value, fail_value])
            current_algo_stats.add_result((current_orien, current_dist),
                                          res_stats)
        
#    current_algo_stats.print_me()
    extended_stats.add_algo_results(current_algo, current_algo_stats)
    
plot_global_results_dataset_performance(extended_stats,
                                        '../' + sim_param.plots_folder,
                                        orien_vector,
                                        dist_vector,
                                        algo_vector)