# -*- coding: utf-8 -*-
"""
Created on Thu Feb  2 13:56:04 2017

@author: maestre
"""

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import random

def create_diverse_trajs(traj):    

    traj_vector = []    
    for nb_traj in range(nb_diverse_trajs):
        tmp_traj = [traj[0]]        
        for traj_wp in traj[1:-1]:            
            orig_traj_x = traj_wp[0]
            orig_traj_y = traj_wp[1]
            orig_traj_z = traj_wp[2]
            
            tmp_traj_wp = []                        
            tmp_traj_wp.append(round(orig_traj_x + random.uniform(-traj_change, traj_change), 
                                     round_value))
            tmp_traj_wp.append(round(orig_traj_y + random.uniform(-traj_change, traj_change), 
                                     round_value))
            tmp_traj_wp.append(round(orig_traj_z + random.uniform(-traj_change, traj_change), 
                                     round_value))
            tmp_traj.append(tmp_traj_wp)
        tmp_traj.append(traj[-1])
        traj_vector.append(tmp_traj)
#        print(nb_traj, tmp_traj)
    return traj_vector

def plot_setup():
    
    # plot figure
    fig = plt.figure()
    fig.clf()
    ax = Axes3D(fig)

    obj_pos = [0.65, 0.1, -0.14]
    obj_side = 0.1
    
    # box init_pos
    ax.bar3d(obj_pos[0] - obj_side/2, 
             obj_pos[1] - obj_side/2, 
             obj_pos[2] - obj_side/2, 
             [.1], [.1], [.1], 
             color='lightgrey',
             alpha=0.2,
             edgecolor='none')

    # robot
    robot_width = .2
    robot_height = .6
    robot_length = .4
    ax.bar3d(-robot_width/2, 
             -robot_length/2, 
             -robot_height/2, 
             robot_width, robot_length, robot_height, 
             color='lightgrey',
             alpha=0.2,
             edgecolor='none')

    # limits
    lim = .3
    ax.set_xlim3d([obj_pos[0]-lim, obj_pos[0]+lim])
    ax.set_ylim3d([obj_pos[1]-lim, obj_pos[1]+lim])
    ax.set_zlim3d([obj_pos[2]-lim, obj_pos[2]+lim])
    
    # labels
    plt.xlabel('xlabel')
    plt.ylabel('ylabel')
#            plt.zlabel('zlabel')
    
    ## view
    ## All commented = diagonal view
#            ax2.view_init(90,180) # top view
#            ax3.view_init(0,0) # front view
#            ax4.view_init(0,270) # left view

#    # plot the big circle points
#    list_x_axis, list_y_axis, list_z_axis = \
#        setup.gen_init_eef(nb_initial_pos,
#                           sim_param.circumference_radio,
#                           obj_pos)
#    list_z_axis = [sim_param.eef_z_value for i in range(len(list_x_axis))]
#    ax.plot(list_x_axis,
#            list_y_axis,
#            list_z_axis,
#            'o',
#            color='red',
#            markersize = 5)

    return ax

def plot_traj(ax, traj):
    ## plot traj from init pos
    eef_pos_vector_x = [pos[0] for pos in traj]
    eef_pos_vector_y = [pos[1] for pos in traj]
    eef_pos_vector_z = [pos[2] for pos in traj]
    ax.plot(eef_pos_vector_x, 
            eef_pos_vector_y, 
            eef_pos_vector_z,
            '-')#,
            #color = 'blue')
    ax.plot(eef_pos_vector_x, 
            eef_pos_vector_y, 
            eef_pos_vector_z,
            '*',                                     
            markersize=3,
            c='grey')            
    
if __name__ == "__main__":
    
    nb_diverse_trajs = 15
    traj_change = 0.05
    round_value = 2
    ax = plot_setup()
    
    traj = [[0.45, -0.05, 0.14],
            [0.55, -0.05, 0.1],
            [0.6, -0.07, 0],
            [0.65, 0.0, -0.14],
            [0.65, 0.1, -0.14]]            
    plot_traj(ax, traj)
            
    traj_vector = create_diverse_trajs(traj)
    traj_vector = [traj] + traj_vector
#    print(traj_vector)        
    
    ax = plot_setup()
    for curr_traj in traj_vector:
        plot_traj(ax, curr_traj)
        
    plt.show()
    
    
    
    
    
    
    
    