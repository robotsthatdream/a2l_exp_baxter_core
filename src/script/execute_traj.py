#!/usr/bin/env python

"""
Created on Mon Nov 30 15:00:16 2015

@author: maestre
"""
import os, sys
lib_path = os.path.abspath(os.path.join('..', 'lib'))
sys.path.append(lib_path)

import pyAgrum as agrum
import random
from math import sin,cos,pi
import numpy as np

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib import colors, colorbar
from matplotlib import gridspec

import discretize as discr
import inference as inf
import parallel as segm

'''
Generate initial eef
'''
def gen_init_eef():
    list_radians = [0]
    i = 0
    while i < 360:
        float_div = 180.0/(i+1)
        list_radians.append(pi/float_div)
        i += 18    
    list_x_axis = []
    list_y_axis = []
    for a in list_radians[1:]:
        list_x_axis.append(cos(a))
        list_y_axis.append(sin(a))    
    tmp = random.randint(0, len(list_x_axis[:-1]))
    if (tmp > len(list_x_axis[:-1])):
        print('Problem while selecting the circle initial eef point',tmp)
    eef_pos = [list_x_axis[tmp], list_y_axis[tmp]]        
    eef_pos = [0.75471, 0.65606, 1.41077]
    return eef_pos

'''
Generate initial pos of the object
'''
def gen_init_box():
    ## random obj pos
    obj_pos = [round(random.uniform(-0.5,0.5),2),
               round(random.uniform(-0.5,0.5),2),
                0]
    obj_side = 0.2

    return obj_pos, obj_side    

'''
Infere and plot new traj
'''
def infere_traj(ie, eef_pos, obj_pos, obj_side, goal):
    discr_traj = []
    eef_traj = [[eef_pos[0], eef_pos[1], 0]]
    obj_moved = False
    nb_iter = 50
    i = 0
    while not obj_moved and i < nb_iter:
        current_eef_x = eef_traj[-1][0]
        current_eef_y = eef_traj[-1][1]
        
        ## compute diff    
        vector_orientation = discr.compute_orientation_discr(
            [current_eef_x,current_eef_y], 
            obj_pos)        
        
        ## infere movement based on diff
        print('\nGOAL :', goal)    
        print("Current DIFF :" , vector_orientation)        
        node_names = ['goal','vector_orient']
        node_values = [goal,vector_orientation]
        next_mov_discr, prob_value, delta_nb_var = inf.infere_mov(bn, ie,                                        
                                        node_names,
                                        node_values)
        print("DELTA :", next_mov_discr, "with probability", prob_value)
        next_eef_x = current_eef_x
        next_eef_y = current_eef_y
        if prob_value > 0.25:
            ## move
            mov_step = 0.05
            if next_mov_discr == 'left':
                next_eef_x -= mov_step
            elif next_mov_discr == 'right':
                next_eef_x += mov_step
            elif next_mov_discr == 'down':
                next_eef_y -= mov_step
            elif next_mov_discr == 'up':
                next_eef_y += mov_step
            elif next_mov_discr == 'right-up':
                next_eef_x += mov_step
                next_eef_y += mov_step    
            elif next_mov_discr == 'right-down':
                next_eef_x += mov_step
                next_eef_y -= mov_step 
            elif next_mov_discr == 'left-up':
                next_eef_x -= mov_step
                next_eef_y += mov_step
            elif next_mov_discr == 'left-down':
                next_eef_x -= mov_step
                next_eef_y -= mov_step
                
            print('New EEF :', next_eef_x, '', next_eef_y)        
            
            ## check if obj was moved
            obj_moved_pose = segm.compute_obj_pos(
                                            obj_pos,
                                            obj_side,                                        
                                            [current_eef_x,current_eef_y,0], 
                                            [next_eef_x,next_eef_y,0])
            obj_moved = True if obj_moved_pose != obj_pos else False  
        
        discr_traj.append(next_mov_discr)        
        eef_traj.append([next_eef_x,next_eef_y, prob_value])
        i += 1
    
    ## print path
    print (discr_traj)
    
    ''' Plot traj ''' 
    fig = plt.gcf()
    fig.set_size_inches(8, 10)
    gs = gridspec.GridSpec(2, 1, height_ratios=[5, 1])
    ax = plt.subplot(gs[0])
    ax2 = plt.subplot(gs[1])    
    
    # set axis limits
    ax.set_xlim(-1.5,1.5)
    ax.set_ylim(-1.5,1.5)
     
    # plot the origin
    origin = Rectangle((obj_pos[0] - obj_side/2, obj_pos[1] - obj_side/2), 
                       obj_side, obj_side, fc="grey")
    ax.add_patch(origin)
     
    # plot the big circle points
    i = 0
    list_radians = []
    while i < 360:
        float_div = 180.0/(i+1)
        list_radians.append(pi/float_div)
        i += 20
    list_x_axis = []
    list_y_axis = []
    for a in list_radians:
        list_x_axis.append(cos(a))
        list_y_axis.append(sin(a))     
    
    ax.plot(list_x_axis,list_y_axis,'*',c='grey')   
    
    ## print goal in the right left corner
    ax.text(-1.25, 1.25, goal.upper(), fontsize=14)
    
    ## for each segment of the new traj    
    nb_bins = 1/delta_nb_var
    for i in range(len(eef_traj)-1):
    
        ## choose color based on probability
        prob_value_segment = eef_traj[i][2]
        if prob_value_segment == 0:
            color = 'white'
        elif prob_value_segment <= nb_bins:
            color = 'yellow'
        elif prob_value_segment <= nb_bins*2:
            color = 'orange'
        elif prob_value_segment <= nb_bins*3:
            color = 'red'
#        elif prob_value_segment <= nb_bins*4:
        else:
            color = 'black'
        
        ## plot the segment
        ax.plot([eef_traj[i][0],eef_traj[i+1][0]],
                [eef_traj[i][1],eef_traj[i+1][1]],
                '-', c=color,
                linewidth=2)
        ax.plot([eef_traj[i][0],eef_traj[i+1][0]],
                [eef_traj[i][1],eef_traj[i+1][1]],
                '.', c='blue', markersize=2)                

    ## draw a colorbar
    colormap_bins = np.arange(0,1.1,nb_bins)
    my_cmap_dist = colors.ListedColormap(['yellow','orange','red','black'])
    my_norm_dist = colors.BoundaryNorm(colormap_bins, my_cmap_dist.N)
    colorbar.ColorbarBase(ax2, cmap=my_cmap_dist,
                                    norm=my_norm_dist,
                                    boundaries=colormap_bins,
                                    ticks=colormap_bins,  # optional
#                                    spacing='proportional',
                                    orientation='horizontal')

plt.show()    

'''
Main
'''
if __name__ == "__main__":

    ## create and learn BN, and setup inference
#    dataset_path = '../generated_files/directed_discr_wps.csv'    
    dataset_path = '../generated_files/random_discr_wps.csv'
    bn = inf.learn_bn('hand-coded', dataset_path)    
#    bn_url = "../generated_files/BN.bif"
#    agrum.saveBN(bn,bn_url)
#    print("BN saved in " + bn_url)

    ## infere and plot a traj    
    ie = agrum.LazyPropagation(bn) 
    eef_pos = gen_init_eef()
    obj_pos, obj_side = gen_init_box()
    goal = 'down'
    infere_traj(ie, eef_pos, obj_pos, obj_side, goal)
        
    print('Done.')    
    