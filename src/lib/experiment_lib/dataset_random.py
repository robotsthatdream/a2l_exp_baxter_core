# -*- coding: utf-8 -*-
"""
@author: maestre
"""
from __future__ import print_function

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import random

import os, sys
run_path = os.path.realpath(os.path.abspath(os.path.join('..', '..')))
sys.path.append(run_path)
lib_path = os.path.realpath(os.path.abspath(os.path.join('..', 'a2l_core_lib')))
sys.path.append(lib_path)
import environment_dynamics as env
import environment_delta as delta
import environment_setup as setup
import discretize_effect as discr
import simulation_parameters as sim_param
import ros_services
        
from collections import OrderedDict

'''
Generate random trajs from each initial position
'''
def create_discr_trajs(nb_initial_pos, 
                       single_init_pos = False,
                       single_pos = 0):

    ''' Get all object pos'''
    obj_pos_dict = OrderedDict()
    for obj_name in sim_param.obj_name_vector:
        obj_pos = ros_services.call_get_model_state(obj_name)
        obj_pos = [round(pos, sim_param.round_value) for pos in obj_pos[0:3]]
        obj_pos_dict[obj_name] = obj_pos

    
    ''' Big circle ''' 
     
    # generate circle positions
    list_x_axis, list_y_axis, list_z_axis = \
        setup.gen_init_eef(nb_initial_pos,
                           sim_param.untucked_left_eef_pos[0] - obj_pos_dict[sim_param.obj_name_vector[0]][0],
                           obj_pos_dict[sim_param.obj_name_vector[0]])
    
    if __name__ == '__main__':
        ''' Plot canvas ''' 
        fig = plt.figure()
        fig.set_size_inches(6, 6)
        ax = fig.add_axes([0, 0, 1, 1])
        
        # set axis limits
        lim = 0.1 + sim_param.untucked_left_eef_pos[0] - obj_pos_dict[sim_param.obj_name_vector[0]][0]
        plt.xlim(obj_pos_dict["cube"][0] - lim,
                 obj_pos_dict["cube"][0] + lim)
        plt.ylim(obj_pos_dict["cube"][1] - lim,
                 obj_pos_dict["cube"][1] + lim)
    #        ax.axis('off')
     
        # plot the origin
        origin = Rectangle((obj_pos_dict["cube"][0]-sim_param.cube_x/2, 
                            obj_pos_dict["cube"][1]-sim_param.cube_y/2), 
                            sim_param.cube_x, sim_param.cube_y, fc="grey")
        ax.add_patch(origin)
         
        # plot the big circle points
        ax.plot(list_x_axis,list_y_axis,'o',c='r')
    
    ''' Generate the trajs '''
    it = 0
    step_length = sim_param.step_length
    step_options = [-step_length, 0, step_length]
    nb_max_box_touched_found = False
    nb_box_touched = 0
    delta_vector = []
    while not nb_max_box_touched_found and it < 50000:
        
        if not single_init_pos:
            loop_range = (0,len(list_x_axis))
        ## if single_init_pos only generate trajs in pos single_pos
        else:
            loop_range = (single_pos, single_pos + 1)        

        ## for each initial position one traj is created each time        
#        for p in range(len(list_x_axis)):
#        for p in range(5,6):    
#        for p in range(0,1):    
        for p in range(loop_range[0], loop_range[1]):

            ## compute each step of the trajectory, called delta
            x_i = [list_x_axis[p]]
            y_i = [list_y_axis[p]]            
            nb_delta = 0
            obj_moved = False
            current_delta_vector = []
            effect = 'None'
            while nb_delta < sim_param.random_max_movs and \
                not obj_moved: # only N movs (delta) allowed
                
                current_x = x_i[-1]
                current_y = y_i[-1]
                if sim_param.semi_random_trajs:
                    new_x = current_x + random.choice(step_options)
                    new_y = current_y + random.choice(step_options)
                else:
                    new_x = current_x + random.uniform(-step_length,step_length)
                    new_y = current_y + random.uniform(-step_length,step_length)
                    
                x_i.append(new_x)
                y_i.append(new_y)
                
                ## compute new box pos
                updated_obj_pos = env.compute_obj_pos(
                                        obj_pos_dict["cube"],
                                        [current_x,current_y,obj_pos_dict["cube"][2]],
                                        [new_x, new_y,obj_pos_dict["cube"][2]])
                updated_obj_pos = updated_obj_pos[1]
                
                obj_moved = True \
                    if updated_obj_pos != obj_pos_dict["cube"] else False
                        
                ## store current delta if there was a move
                if current_x != new_x or \
                    current_y != new_y:
                        
                    current_delta = delta.Delta(
                                effect,
                                current_x,current_y,0,
                                new_x, new_y,0,
                                [obj_pos_dict["cube"][0], 
                                 obj_pos_dict["cube"][1],
                                 obj_pos_dict["cube"][2], ## constant
                                 updated_obj_pos[0], 
                                 updated_obj_pos[1],
                                 obj_pos_dict["cube"][2]]) ## constant
#                                ,
#                                obj_moved)
                    current_delta_vector.append(current_delta)
#                    print(len(current_delta_vector))
                
                if obj_moved:
                    ## stop generating wps if max trajs inferred
                    nb_box_touched += 1
                    if nb_box_touched == \
                        nb_initial_pos * len(sim_param.effect_values):
                        nb_max_box_touched_found = True
                    ## compute related effect                                            
                    effect = discr.compute_effect(obj_pos_dict["cube"],
                                                  updated_obj_pos)
#                    print(updated_obj_pos, effect)
                    for d in current_delta_vector:
                        d.set_effect(effect)
                    
                nb_delta += 1
            
            ## print traj
            if __name__ == "__main__" and obj_moved:
                ax.plot(x_i, y_i, '-')
                ax.plot(x_i, y_i, '*', c='grey')
          
            ## only store trajs contacting the box
            if obj_moved :
                delta_vector = delta_vector + current_delta_vector
#            print('len delta_vector', len(delta_vector))
                
        it += 1

    if __name__ == "__main__":
        plt.show()
    
    return delta_vector
    


'''
Generate random trajs prodducing a specific effect
'''
def create_effect_trajs(nb_initial_pos,
                       single_pos,
                       desired_effect,
                       dataset_size):
    
    ''' Big circle ''' 
     
    # generate circle positions
    list_x_axis, list_y_axis, list_z_axis = \
        setup.gen_init_eef(nb_initial_pos)
    
#    if sim_param.print_discr_random_dataset:
    if __name__ == '__main__':
        ''' Plot canvas ''' 
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
         
        # plot the big circle points
        ax.plot(list_x_axis,list_y_axis,'o',c='r')
    
    ''' Generate the trajs '''
    it = 0
    step_length = sim_param.step_length
    step_options = [-step_length, 0, step_length]
    delta_vector = []
    desired_effect_achieved = False
    current_nb_desired_effect = 0
    
    while not desired_effect_achieved: # and \
#          it < 50000:     
        
        ## compute each step of the trajectory, called delta
        x_i = [list_x_axis[single_pos]]
        y_i = [list_y_axis[single_pos]]            
        nb_delta = 0
        obj_moved = False
        current_delta_vector = []
        effect = 'None'
        ## new trajectory
        while nb_delta < sim_param.random_max_movs and \
            not obj_moved: # only N movs (delta) allowed
            
            current_x = x_i[-1]
            current_y = y_i[-1]                
            new_x = current_x + random.uniform(-step_length*2,step_length*2)
            new_y = current_y + random.uniform(-step_length*2,step_length*2)
#                new_x = current_x + random.choice(step_options)
#                new_y = current_y + random.choice(step_options)
            x_i.append(new_x)
            y_i.append(new_y)
            
            ## compute new box pos
            updated_obj_pos = env.compute_obj_pos(
                                    [current_x,current_y,0],
                                    [new_x, new_y,0])     
            
            obj_moved = True \
                if updated_obj_pos != sim_param.obj_pos else False                    
            
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
                current_delta_vector.append(current_delta)
#                    print(len(current_delta_vector))
            
            if obj_moved:
                ## compute related effect
                effect = discr.compute_effect(updated_obj_pos)
                for d in current_delta_vector:
                    d.set_effect(effect)
                
            nb_delta += 1
        
        ## print traj
        if __name__ == "__main__" and obj_moved and effect==desired_effect:
            ax.plot(x_i, y_i, '-')
            ax.plot(x_i, y_i, '*', c='grey')
      
        ## only store trajs contacting the box
        if obj_moved and effect==desired_effect:
            current_nb_desired_effect += 1
            delta_vector = delta_vector + current_delta_vector            
            if current_nb_desired_effect == \
                sim_param.nb_desired_effect_reproduced:                    
#                dataset_size * 0.01:
                    print('dataset_size', dataset_size)
                    desired_effect_achieved = True                
        it += 1

    if __name__ == "__main__":
        plt.show()
        print(current_nb_desired_effect)
    
    return delta_vector
    

'''
Test
'''
#if sim_param.test_random_dataset:
if __name__ == "__main__":
    create_discr_trajs(8, False, 2)
#    create_effect_trajs(8, 0, 'right')