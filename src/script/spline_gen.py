# -*- coding: utf-8 -*-
"""
Created on Fri Mar  4 10:17:38 2016

@author: maestre
"""

from math import sin,cos,pi
import numpy as np

import parallel as segm
import discretize as discr
'''
Input : initial eef position + final segment to reach
Output : discretized dataset of trajectories to move the obj
'''
def create_discr_spline_to_effect(obj_pos, obj_side):
    
    ''' Big circle ''' 
    list_radians = [0]
     
    # generate circle positions
    i = 0
    while i < 360:
        float_div = 180.0/(i+1)
        list_radians.append(pi/float_div)
        i += 20
         
    # list of coordinates for each point
    list_x_axis = []
    list_y_axis = []
     
    # calculate coordinates 
    # and append to above list
    for a in list_radians[1:]:
        list_x_axis.append(cos(a))
        list_y_axis.append(sin(a))
    
    #''' Small circle ''' 
    #list_subpoints = []
    #for p in range(len(list_x_axis)):        
    #    list_rads_subpoints = [0]    
    #     
    #    i = 0
    #    while i < 360:
    #        float_div = 180.0/(i+1)
    #        list_rads_subpoints.append(pi/float_div)
    #        i += 45
    #         
    #    # list of coordinates for each point
    #    list_sub_x_axis = []
    #    list_sub_y_axis = []
    #     
    #    # calculate coordinates 
    #    # and append to above list
    #    for a in list_rads_subpoints[1:]:
    #        list_sub_x_axis.append(list_x_axis[p] + cos(a)/5)
    #        list_sub_y_axis.append(list_y_axis[p] + sin(a)/5)        
    #
    #    list_subpoints.append([list_sub_x_axis, list_sub_y_axis])    
    
#    ''' Plot canvas ''' 
#    fig = plt.gcf()
#    ax = fig.add_subplot(111)
#    fig.set_size_inches(8, 8)
#    
#    # set axis limits
#    plt.xlim(-1.5,1.5)
#    plt.ylim(-1.5,1.5)
#     
#    # plot the origin
#    #plt.plot(0,0,'o',c='black')
#    origin = Rectangle((-obj_side/2, -obj_side/2), obj_side, obj_side, fc="grey")
#    ax.add_patch(origin)
#     
#    # plot the big circle points
#    ax.plot(list_x_axis,list_y_axis,'o',c='r')
#    
#    # plot the small circle points
#    #for p in range(len(list_x_axis)):
    
    ## generate trajs
    trajs = []
    #z_values = np.zeros(len(list_subpoints[0]))
    z_values = np.zeros(len(list_x_axis))
    
    ''' Generate the trajs ''' 
    for p in range(len(list_x_axis)):    
#    for p in range(9,10):    
    
    #    plt.plot(list_subpoints[p][0],list_subpoints[p][1],'o',c='b')    
        #    for subp in range (0, len(list_subpoints[p][0])):
        ##    for subp in range (0,1):
        #        traj  = []
        #        ''' Spline ''' 
        #        x = [list_x_axis[p],list_subpoints[p][0][subp], obj_pos[0]]
        #        y = [list_y_axis[p],list_subpoints[p][1][subp], obj_pos[0]]
        #        
        #        tck,u = interpolate.splprep([x,y],k = 2)
        #        x_i,y_i = interpolate.splev(np.linspace(0,1,20),tck)
        #        ax.plot(x_i, y_i, '-')
        ##        plt.plot(x_i, y_i, '*', c='grey')        
        ##        for v in range(len(x_i)):
        ##            traj_wp = [x_i[v], y_i[v], x_i[v] + y_i[v]] 
        ##            traj.append(traj_wp)
        ##        trajs.append(traj)
    
        
        x_i = np.linspace(list_x_axis[p], obj_pos[0], 15)
        y_i = np.linspace(list_y_axis[p], obj_pos[1], 15)
#        ax.plot(x_i, y_i, '-', c='red')
#        ax.plot(x_i, y_i, '*', c='grey')
    
        traj = []
        for v in range(len(x_i)):
            traj_wp = [x_i[v], y_i[v], x_i[v] + y_i[v]]
            traj.append(traj_wp)
        trajs.append(traj)
    
    ''' Compute the final obj position ''' 
    # for each mov check the interaction of it to the obj
    posRot_trajs = []
    for traj in trajs:
        obj_moved = False
        posRot_traj = []
        for wp_pos in range(len(traj)-1):
            if not obj_moved:
                obj_moved_pose = segm.compute_obj_pos(
                                        obj_pos,
                                        obj_side,                                        
                                        traj[wp_pos], 
                                        traj[wp_pos+1])         
            posRot_traj.append([traj[wp_pos],obj_moved_pose])
            obj_moved = True if obj_moved_pose != obj_pos else False            
        posRot_trajs.append(posRot_traj)

    ''' Write the trajs dataset ''' 
    filename = 'raw_traj_dataset.csv'
    file = open(filename, 'w')
    round_value = 5
    for traj in posRot_trajs:
        for wp in traj:
            eef_pos = wp[0]
            obj_pos = wp[1]
            file.write(str(round(eef_pos[0],round_value)))
            file.write(' ')
            file.write(str(round(eef_pos[1],round_value)))
            file.write(' ')
            file.write(str(round(eef_pos[2],round_value)))
            file.write(' ')
            file.write(str(round(obj_pos[0],round_value)))
            file.write(' ')
            file.write(str(round(obj_pos[1],round_value)))
            file.write(' ')
            file.write(str(round(obj_pos[2],round_value)))
            file.write(' ')                
            file.write('\n') 
    
    ''' Discretize the dataset '''
    discr_traj_vector = discr.discretize_trajs(posRot_trajs)
    
    ''' Write the trajs discretized dataset
    discr_traj_vector = [discr_traj]
    discr_traj = [discr_wp]
    discr_wp = [goal, vector_orient, delta_orient]
    vector_orient = up, dowm, left, right, left-up, etc
    delta_orient = up, dowm, left, right
    '''
    filename = 'discr_wps.csv'
    file = open(filename, 'w')    
    file.write('goal')
    file.write(',')
    file.write('vector_orient')
    file.write(',')
    file.write('delta_orient')
    file.write('\n')    
    
    traj_pos = 0
    mov_step = 0.1
    for discr_traj in discr_traj_vector:
        for discr_wp in discr_traj:
            ## write to file
            for q in range(len(discr_wp)):
                file.write(discr_wp[q])
                if q != (len(discr_wp)-1):
                    file.write(',')
            file.write('\n') 
                
            ## compute delta in X Y Z
            wp_eef_pos = posRot_trajs[traj_pos][0][0] # X Y Z pos of WP
                                    # start at init pos of each traj
            delta_x = [wp_eef_pos[0]]
            delta_y = [wp_eef_pos[1]]
            new_x = wp_eef_pos[0]
            new_y = wp_eef_pos[1]
            if discr_wp[2] == 'right':
                new_x += mov_step
            elif discr_wp [2] == 'left':
                new_x -= mov_step
            elif discr_wp [2] == 'up':
                new_y += mov_step            
            elif discr_wp [2] == 'down':
                new_y -= mov_step
            delta_x.append(new_x)
            delta_y.append(new_y)
#            ax.plot(delta_x, delta_y, '-', c='blue')
#            ax.plot(delta_x, delta_y, '*', c='grey')
            wp_eef_pos[0] = new_x
            wp_eef_pos[1] = new_y            
        traj_pos += 1
    file.close()
        
    # show the plot
#    plt.show()
    
    return discr_traj_vector

#create_discr_trajs()