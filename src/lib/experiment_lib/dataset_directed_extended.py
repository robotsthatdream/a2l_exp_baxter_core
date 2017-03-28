# -*- coding: utf-8 -*-
"""
@author: maestre
"""

from __future__ import print_function

#from math import sin,cos,pi
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle

import os, sys
run_path = os.path.realpath(os.path.abspath(os.path.join('..', '..')))
sys.path.append(run_path)
lib_path = os.path.realpath(os.path.abspath(os.path.join('..', 'a2l_core_lib')))
sys.path.append(lib_path)
import environment_setup as setup
import environment_dynamics as env
#import discretize as discr
import environment_delta as delta
import simulation_parameters as sim_param

class Traj:
    def __init__(self, traj_x, traj_y, effect):
        self.traj_x = traj_x
        self.traj_y = traj_y
        self.effect = effect
        
    def get_traj_x(self):
        return self.traj_x
    def get_traj_y(self):
        return self.traj_y        
    def get_effect(self):
        return self.effect
        
class Predef_pos:
    def __init__(self, x, y, traj_vector):
        self.x = x
        self.y = y
        self.traj_vector = traj_vector
        
    def get_x(self):
        return self.x
    def get_y(self):
        return self.y
    def get_traj_vector(self):
        return self.traj_vector


'''
Input : center point of obj (X Y Z) + obj size
Output : discretized dataset of trajectories to move the obj
'''
def create_discr_trajs(nb_initial_pos):
    
    ''' Big circle '''         
    # generate circle positions
    list_x_axis, list_y_axis, list_z_axis = \
        setup.gen_init_eef(nb_initial_pos)
 
    if sim_param.print_directed_dataset_extended:  
        ''' Plot canvas ''' 
        fig = plt.gcf()
        fig.set_size_inches(5,5)
        ax = fig.add_axes([0, 0, 1, 1])        
        
        # set axis limits
        plt.xlim(-1.2,1.2)
        plt.ylim(-1.2,1.2)
         
        # plot the origin
        origin = Rectangle((sim_param.obj_pos[0]-sim_param.obj_side/2, 
                            sim_param.obj_pos[1]-sim_param.obj_side/2), 
                           sim_param.obj_side, sim_param.obj_side, fc="grey")
        ax.add_patch(origin)
    #    ax.axis('off')
        
        # plot the big circle points
        ax.plot(list_x_axis,list_y_axis,'o',c='r')
    
    ''' Generate the trajs ''' 
    
    ## Generate predefined mid pos around the box
    obj_pos = sim_param.obj_pos
    mid_pos_dist = sim_param.obj_side + sim_param.obj_side/2
    
    pos_t_l = [obj_pos[0] - mid_pos_dist, obj_pos[1] + mid_pos_dist]
    pos_t_c = [obj_pos[0], obj_pos[1] + mid_pos_dist]
    pos_t_r = [obj_pos[0] + mid_pos_dist, obj_pos[1] + mid_pos_dist]
    pos_m_l = [obj_pos[0] - mid_pos_dist, obj_pos[1]]
    pos_m_r = [obj_pos[0] + mid_pos_dist, obj_pos[1]]
    pos_d_l = [obj_pos[0] - mid_pos_dist, obj_pos[1] - mid_pos_dist]
    pos_d_c = [obj_pos[0], obj_pos[1] - mid_pos_dist]
    pos_d_r = [obj_pos[0] + mid_pos_dist, obj_pos[1] - mid_pos_dist]
    
    ## MID_POINT = [POS, [related TRAJS]    ## Generate predefined trajs to the center of the obj
    ## First, the directed trajs, from the sides
#    trajs_steps = 3
    
    ''' top-center '''    
    trajs_steps = setup.compute_nb_wps(pos_t_c, obj_pos[-1]) 
    traj_t_c_y = list(np.linspace(pos_t_c[1], 
                             obj_pos[1], 
                             trajs_steps))
    traj_t_c_x = [obj_pos[0] for i in range(len(traj_t_c_y))]                             
    traj_t_c = Traj(traj_t_c_x, traj_t_c_y, 'down')
    t_c = Predef_pos(pos_t_c[0], pos_t_c[1], [traj_t_c])
    
    ''' mid_left '''
    traj_m_l_x = list(np.linspace(pos_m_l[0], 
                             obj_pos[0], 
                             trajs_steps))  
    traj_m_l_y = [obj_pos[1] for i in range(len(traj_m_l_x))]    
    traj_m_l = Traj(traj_m_l_x, traj_m_l_y, 'right')
    m_l = Predef_pos(pos_m_l[0], pos_m_l[1], [traj_m_l])
    
    ''' mid_right '''
    traj_m_r_x = list(np.linspace(pos_m_r[0],
                                  obj_pos[0],                             
                                  trajs_steps))  
    traj_m_r_y = [obj_pos[1] for i in range(len(traj_m_r_x))]       
    traj_m_r = Traj(traj_m_r_x, traj_m_r_y, 'left')
    m_r = Predef_pos(pos_m_r[0], pos_m_r[1], [traj_m_r])
    
    ''' down-center '''
    traj_d_c_y = list(np.linspace(pos_d_c[1],
                                  obj_pos[1],                              
                                  trajs_steps))
    traj_d_c_x = [obj_pos[0] for i in range(len(traj_d_c_y))]
    traj_d_c = Traj(traj_d_c_x, traj_d_c_y, 'up')
    d_c = Predef_pos(pos_d_c[0], pos_d_c[1], [traj_d_c])
    
    ## Now the extended trajs from the corners to the sides,
    ## and from there to the center of the box
    
    ''' top-left '''
    ## top-left + top-center
    traj_t_l_t_c_x = list(np.linspace(pos_t_l[0], 
                                 pos_t_c[0], 
                                 trajs_steps)) + traj_t_c_x
            
#    traj_t_l_t_c_y = [pos_t_l[1] for i in range(trajs_steps)] + traj_t_c_y
    traj_t_l_t_c_y = [pos_t_l[1] for i in range(len(traj_t_l_t_c_x)-len(traj_t_c_y))] + traj_t_c_y            
    traj_t_l_t_c = Traj(traj_t_l_t_c_x, traj_t_l_t_c_y, 'down')
#        ax.plot(traj_t_l_t_c_x, traj_t_l_t_c_y, '-')        
    
    ## top-left + mid-left
#    traj_t_l_m_l_x = [pos_t_l[0] for i in range(trajs_steps)] + traj_m_l_x
    traj_t_l_m_l_y = list(np.linspace(pos_t_l[1], 
                                 pos_m_l[1], 
                                 trajs_steps)) + traj_m_l_y                    
    traj_t_l_m_l_x = [pos_t_l[0] for i in range(len(traj_t_l_m_l_y) - len(traj_m_l_x) )] + traj_m_l_x
    traj_t_l_m_l = Traj(traj_t_l_m_l_x, traj_t_l_m_l_y, 'right')
#        ax.plot(traj_t_l_m_l_x, traj_t_l_m_l_y, '-')
    
    t_l = Predef_pos(pos_t_l[0], pos_t_l[1], [traj_t_l_t_c, traj_t_l_m_l])
    
    ''' top-right '''
    ## top-right + top-center
    traj_t_r_t_c_x = list(np.linspace(pos_t_r[0], 
                                 pos_t_c[0], 
                                 trajs_steps)) + traj_t_c_x            
#    traj_t_r_t_c_y = [pos_t_r[1] for i in range(trajs_steps)] + traj_t_c_y
    traj_t_r_t_c_y = [pos_t_r[1] for i in range(len(traj_t_r_t_c_x) - len(traj_t_c_y) )] + traj_t_c_y
    traj_t_r_t_c = Traj(traj_t_r_t_c_x, traj_t_r_t_c_y, 'down')
#        ax.plot(traj_t_r_t_c_x, traj_t_r_t_c_y, '-')
    
    ## top-right + mid-right
#    traj_t_r_m_r_x = [pos_t_r[0] for i in range(trajs_steps)] + traj_m_r_x
    traj_t_r_m_r_y = list(np.linspace(pos_t_r[1], 
                                 pos_m_r[1], 
                                 trajs_steps)) + traj_m_r_y                    
    traj_t_r_m_r_x = [pos_t_r[0] for i in range(len(traj_t_r_m_r_y) - len(traj_m_r_x))] + traj_m_r_x
    traj_t_r_m_r = Traj(traj_t_r_m_r_x, traj_t_r_m_r_y, 'left')
#        ax.plot(traj_t_r_m_r_x, traj_t_r_m_r_y, '-')    
    
    t_r = Predef_pos(pos_t_r[0], pos_t_r[1], [traj_t_r_t_c, traj_t_r_m_r])
    
    ''' down-left '''
    ## down-left + down-center
    traj_d_l_d_c_x = list(np.linspace(pos_d_l[0], 
                                 pos_d_c[0], 
                                 trajs_steps)) + traj_d_c_x
            
#    traj_d_l_d_c_y = [pos_d_l[1] for i in range(trajs_steps)] + traj_d_c_y
    traj_d_l_d_c_y = [pos_d_l[1] for i in range(len(traj_d_l_d_c_x) - len(traj_d_c_y))] + traj_d_c_y
    traj_d_l_d_c = Traj(traj_d_l_d_c_x, traj_d_l_d_c_y, 'up')
#        ax.plot(traj_d_l_d_c_x, traj_d_l_d_c_y, '-')
    
    ## down-left + mid-left
#    traj_d_l_m_l_x = [pos_d_l[0] for i in range(trajs_steps)] + traj_m_l_x
    traj_d_l_m_l_y = list(np.linspace(pos_d_l[1], 
                                 pos_m_l[1], 
                                 trajs_steps)) + traj_m_l_y
    traj_d_l_m_l_x = [pos_d_l[0] for i in range(len(traj_d_l_m_l_y) - len(traj_m_l_x))] + traj_m_l_x
    traj_d_l_m_l = Traj(traj_d_l_m_l_x, traj_d_l_m_l_y, 'right')
#        ax.plot(traj_d_l_m_l_x, traj_d_l_m_l_y, '-')
                                 
    d_l = Predef_pos(pos_d_l[0], pos_d_l[1], [traj_d_l_d_c, traj_d_l_m_l])
                                 
    ''' down-right '''
    ## down-right + down-center
    traj_d_r_d_c_x = list(np.linspace(pos_d_r[0], 
                                 pos_d_c[0], 
                                 trajs_steps)) + traj_d_c_x
            
#    traj_d_r_d_c_y = [pos_d_r[1] for i in range(trajs_steps)] + traj_d_c_y
    traj_d_r_d_c_y = [pos_d_r[1] for i in range(len(traj_d_r_d_c_x) - len(traj_d_c_y) )] + traj_d_c_y
    traj_d_r_d_c = Traj(traj_d_r_d_c_x, traj_d_r_d_c_y, 'up')
#        ax.plot(traj_d_r_d_c_x, traj_d_r_d_c_y, '-')
    
    ## down-right + mid-right
#    traj_d_r_m_r_x = [pos_d_r[0] for i in range(trajs_steps)] + traj_m_r_x
    traj_d_r_m_r_y = list(np.linspace(pos_d_r[1], 
                                 pos_m_r[1], 
                                 trajs_steps)) + traj_m_r_y

    traj_d_r_m_r_x = [pos_d_r[0] for i in range(len(traj_d_r_m_r_y) - len(traj_m_r_x) )] + traj_m_r_x                    
    traj_d_r_m_r = Traj(traj_d_r_m_r_x, traj_d_r_m_r_y, 'left')
#        ax.plot(traj_d_r_m_r_x, traj_d_r_m_r_y, '-')
    
    d_r = Predef_pos(pos_d_r[0], pos_d_r[1], [traj_d_r_d_c, traj_d_r_m_r])     
    
    ## Set of available predefined pos and trajs
    predefined_pos_traj_vector = [t_l, t_c, t_r, m_l, m_r, d_l, d_c, d_r]        
           
    ## From each init pos, create trajs to the predefined points
    ## If these trajs do not contact the box, extend them with the 
    ## predefined trajs from each predefined point
    delta_vector = []
    save_fig_id = 0
    for init_pos in range(len(list_x_axis)):
#    for init_pos in range(0,1):
        ## for each predefined pos around the obj

#        if init_pos == 0:
#            t_l = Predef_pos(pos_t_l[0], pos_t_l[1], [traj_t_l_t_c])
#            t_r = Predef_pos(pos_t_r[0], pos_t_r[1], [])
#            d_l = Predef_pos(pos_d_l[0], pos_d_l[1], [traj_d_l_d_c, traj_d_l_m_l])
#            d_r = Predef_pos(pos_d_r[0], pos_d_r[1], [traj_d_r_m_r])     
#    
#        
#        elif init_pos == 1:
#            t_l = Predef_pos(pos_t_l[0], pos_t_l[1], [traj_t_l_t_c])
#            t_r = Predef_pos(pos_t_r[0], pos_t_r[1], [traj_t_r_t_c])
#            d_l = Predef_pos(pos_d_l[0], pos_d_l[1], [traj_d_l_m_l])
#            d_r = Predef_pos(pos_d_r[0], pos_d_r[1], [traj_d_r_m_r])     
#                
#            
#        elif init_pos == 2:
#            t_l = Predef_pos(pos_t_l[0], pos_t_l[1], [])
#            t_r = Predef_pos(pos_t_r[0], pos_t_r[1], [traj_t_r_t_c])
#            d_l = Predef_pos(pos_d_l[0], pos_d_l[1], [traj_d_l_m_l])
#            d_r = Predef_pos(pos_d_r[0], pos_d_r[1], [traj_d_r_d_c, traj_d_r_m_r])     
#                
#        predefined_pos_traj_vector = [t_l, t_c, t_r, m_l, m_r, d_l, d_c, d_r]             
     
     
        for current_pred_pos_traj in predefined_pos_traj_vector:
            ## create a traj to it
            trajs_steps = setup.compute_nb_wps([list_x_axis[init_pos],
                                                list_y_axis[init_pos]],
                                                [current_pred_pos_traj.get_x(),
                                                 current_pred_pos_traj.get_y()])
            traj_x = list(np.linspace(list_x_axis[init_pos], 
                                      current_pred_pos_traj.get_x(),
                                      trajs_steps))
            traj_y = list(np.linspace(list_y_axis[init_pos], 
                                      current_pred_pos_traj.get_y(),
                                      trajs_steps))
            
#            ax.plot(traj_x, traj_y, '-')
#            ax.plot(traj_x, traj_y, 'b*')
                            
            ## Check contact of each wp with object
            current_init_traj = list(zip(traj_x, traj_y))
            obj_moved = False
            current_wp_pos = 0
            while not obj_moved and current_wp_pos < len(current_init_traj)-1:
                
                current_x = current_init_traj[current_wp_pos][0]
                current_y = current_init_traj[current_wp_pos][1]
                
                next_x = current_init_traj[current_wp_pos+1][0]
                next_y = current_init_traj[current_wp_pos+1][1]
                
                ## compute new box pos
                updated_obj_pos = env.compute_obj_pos(
                                        [current_x,current_y,0],
                                        [next_x, next_y,0])             
                obj_moved = True \
                    if updated_obj_pos != sim_param.obj_pos else False
                current_wp_pos += 1
        
            ## If no interaction with the obj, extend the traj
            ## with the predefined ones            
            if not obj_moved:
                current_extended_traj_vector = []
                pred_traj_vector = current_pred_pos_traj.get_traj_vector()                
                for pred_traj in pred_traj_vector:
                    current_extended_traj = Traj(
                        traj_x + pred_traj.get_traj_x(),
                        traj_y + pred_traj.get_traj_y(),
                        pred_traj.get_effect())
                    current_extended_traj_vector.append(
                        current_extended_traj)
                    
                    if sim_param.print_directed_dataset_extended:# and init_pos == 7:
                        ax.plot(current_extended_traj.get_traj_x(), 
                                current_extended_traj.get_traj_y(), '-*')
#                    plt.savefig('save_fig_' + str(save_fig_id) + '.png')
#                    save_fig_id += 1
            
                ## For each created extended trajectory, split it up into deltas
                current_delta_vector = []
                for ext_traj in current_extended_traj_vector:
                    ext_traj_x = ext_traj.get_traj_x()           
                    ext_traj_y = ext_traj.get_traj_y()
                    for ext_wp_pos in range(len(ext_traj_x)-1):                                                                        
                        current_x = ext_traj_x[ext_wp_pos]
                        current_y = ext_traj_y[ext_wp_pos]                       
                        next_x = ext_traj_x[ext_wp_pos+1]
                        next_y = ext_traj_y[ext_wp_pos+1]
                        
                        updated_obj_pos = env.compute_obj_pos(
                                        [current_x,current_y,0],
                                        [next_x, next_y,0])
                        
                        current_delta = delta.Delta(
                            ext_traj.get_effect(),
                            current_x,current_y,0,
                            next_x, next_y,0,
                            sim_param.obj_pos[0], sim_param.obj_pos[1],0,
                            updated_obj_pos[0], updated_obj_pos[1],0,
                            True) ## Extended trajs are always moving the obj
                            
                        current_delta_vector.append(current_delta)
                delta_vector += current_delta_vector
        
    if sim_param.print_directed_dataset_extended:
        plt.show()
    
    return delta_vector

'''
Test
'''
if sim_param.test_directed_dataset_extended:
    create_discr_trajs(sim_param.nb_min_init_pos)