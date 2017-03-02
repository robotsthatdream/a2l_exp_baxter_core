# -*- coding: utf-8 -*-
"""
@author: maestre
"""

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import random

import os, sys
run_path = os.path.realpath(os.path.abspath(os.path.join('..', '..')))
sys.path.append(run_path)
lib_path = os.path.realpath(os.path.abspath(os.path.join('..', 'a2l_core_lib')))
sys.path.append(lib_path)
import ros_services
import simulation_parameters as sim_param
from numpy import linspace
import copy

from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Circle
import mpl_toolkits.mplot3d.art3d as art3d
import numpy as np
from scipy.linalg import norm

def plot_setup(obj_vector):
    
    # plot figure
    fig = plt.figure(figsize=(7,7))
    fig.clf()
    ax = Axes3D(fig)
    
    # robot
    robot_width = .2
    robot_height = .6
    robot_length = .4
    ax.bar3d(-robot_width/2, 
             -robot_length/2, 
             -robot_height/2, 
             robot_width, robot_length, robot_height, 
             color='red',
             alpha=0.2,
             edgecolor='none')                           
             
    # labels
    plt.xlabel('x-axis')
    plt.ylabel('y-axis')
    
    pos_cube = obj_vector[0]
    tmp_obj_vector = [[pos_cube]]        
    
    if push_cylinder:
        pos_cylinder = obj_vector[1]
        tmp_obj_vector[0].append(pos_cylinder)
        
    plot_objects(ax, tmp_obj_vector)

    return ax
    
def plot_objects(ax, obj_vector):
    # box init_pos
    pos_cube = [obj_vector[0][0][0],
                obj_vector[0][0][1],
                obj_vector[0][0][2]]

    ax.bar3d(pos_cube[0] - 0.085/2, 
             pos_cube[1] - 0.07/2, 
             pos_cube[2] - 0.08, 
             [.085], [.07], [.08], 
             color='green',
             alpha=0.2,
             edgecolor='none')
             
    # limits
    obj_pos = pos_cube
    lim = .3
    ax.set_xlim3d([(obj_pos[0]-lim), (obj_pos[0]+lim)])
    ax.set_ylim3d([(obj_pos[1]-lim), (obj_pos[1]+lim*1)])
    ax.set_zlim3d([(obj_pos[2]-lim), (obj_pos[2]+lim)])             
             
    if push_cylinder:
        
        pos_cylinder = [obj_vector[0][1][0],
                        obj_vector[0][1][1],
                        obj_vector[0][1][2]]                
        
        ## http://stackoverflow.com/questions/39822480/plotting-a-solid-cylinder-centered-on-a-plane-in-matplotlib
        height = 0.09
        R = 0.035
        obj_pos = pos_cylinder
        p0 = np.array([obj_pos[0], obj_pos[1], obj_pos[2] - height]) #point at one end
        p1 = np.array([obj_pos[0], obj_pos[1], obj_pos[2]]) #point at other end        
        v = p1 - p0
        mag = norm(v)
        v = v / mag
        not_v = np.array([1, 0, 0])
        if (v == not_v).all():
            not_v = np.array([0, 1, 0])
        n1 = np.cross(v, not_v)
        n1 /= norm(n1)
        n2 = np.cross(v, n1)
        t = np.linspace(0, mag, 2)
        theta = np.linspace(0, 2 * np.pi, 100)
        rsample = np.linspace(0, R, 2)
        t, theta2 = np.meshgrid(t, theta)
        rsample,theta = np.meshgrid(rsample, theta)
        # "Tube"
        X, Y, Z = [p0[i] + v[i] * t + R * np.sin(theta2) * n1[i] + R * np.cos(theta2) *       n2[i] for i in [0, 1, 2]]
        # "Bottom"
        X2, Y2, Z2 = [p0[i] + rsample[i] * np.sin(theta) * n1[i] + rsample[i] * np.cos(theta) * n2[i] for i in [0, 1, 2]]
        # "Top"
        X3, Y3, Z3 = [p0[i] + v[i]*mag + rsample[i] * np.sin(theta) * n1[i] + rsample[i] * np.cos(theta) * n2[i] for i in [0, 1, 2]]        
        ax.plot_surface(X, Y, Z, color='blue', linewidth=0, alpha=0.2)
        ax.plot_surface(X2, Y2, Z2, color='blue', linewidth=0, alpha=0.2)
        ax.plot_surface(X3, Y3, Z3, color='blue', linewidth=0, alpha=0.2)             


def plot_traj(ax, traj_vector):
    
    color_vector = []
    for traj in traj_vector:        
        ## plot traj from init pos
        color = np.random.rand(3,1)
        color_vector.append(color)
    
        eef_pos_vector_x = [pos[0] for pos in traj]
        eef_pos_vector_y = [pos[1] for pos in traj]
        eef_pos_vector_z = [pos[2] for pos in traj]
        ax.plot(eef_pos_vector_x, 
                eef_pos_vector_y, 
                eef_pos_vector_z,
                '-'),
#                color = 'black') #color)
        ax.plot(eef_pos_vector_x, 
                eef_pos_vector_y, 
                eef_pos_vector_z,
                '*',                                     
                markersize=3,
                c='grey')
                
    return color_vector
            
def plot_object_change(ax, object_vector, color_vector):

    ## put positions of each object in same list
    tmp_obj_pos_vector = []
    for traj in object_vector:
        for i in range(nb_objects):
            tmp = []
            for pos in range(i,len(traj),nb_objects):
                tmp.append(traj[pos])
            tmp_obj_pos_vector.append(tmp)
        
#        for obj_pos_list in tmp_obj_pos_vector:
        for i in range(nb_objects):
            ## plot traj from init pos
            traj = tmp_obj_pos_vector[i]
            if i == 0 :
                color = 'blue'
            else:
                color = 'red'
#            color_obj_traj = color_vector[i]
            obj_pos_vector_x = [pos[0] for pos in traj]
            obj_pos_vector_y = [pos[1] for pos in traj]
            obj_pos_vector_z = [pos[2] for pos in traj]
            ax.plot(obj_pos_vector_x, 
                    obj_pos_vector_y, 
                    obj_pos_vector_z,
                    '-',
                    linewidth=3,
                    color = color)
#                    color = color_obj_traj)
            ax.plot(obj_pos_vector_x, 
                    obj_pos_vector_y, 
                    obj_pos_vector_z,
                    marker='<',                                     
                    markersize=3,
                    c='grey')


def get_environment_pos():
    
    object_vector = []
    
    ''' Get obj and eef states '''
    eef_pos = ros_services.call_get_eef_pose('left')
    eef_pos = [round(pos, sim_param.round_value) for pos in eef_pos[0:3]]
    print("eef_pos", eef_pos)

    cube_pos = ros_services.call_get_model_state(obj_name_vector[0])
    cube_pos = [round(pos, sim_param.round_value + 1) for pos in cube_pos[0:3]]
#    cube_pos[2] = round(cube_pos[2] + cube_height/2 + 0.015, 
#                        sim_param.round_value + 1)
    cube_pos[2] = -0.09
    object_vector.append(cube_pos)
    print('cube pos', cube_pos)
    
    if push_cylinder:
        cylinder_pos = ros_services.call_get_model_state(obj_name_vector[1])        
        cylinder_pos = [round(pos, sim_param.round_value + 1) for pos in cylinder_pos[0:3]]
#        cylinder_pos[2] = round(cylinder_pos[2] + cylinder_height/2 + 0.02, 
#                                sim_param.round_value + 1)
        cylinder_pos[2] = -0.08
        object_vector.append(cylinder_pos)
        print('cylinder_pos', cylinder_pos)
    
    return eef_pos, object_vector 
            

def create_diverse_trajs(traj,
                         obj_pos_vector,
                         effect):

        tmp_traj = [] ## [eef_pos eef_orien obj_pos obj_orien] = [float]
        
        zero_vector = [0,0,0]
        
        for traj_wp in traj:
            orig_traj_x = round(traj_wp[0], sim_param.round_value+2)
            orig_traj_y = round(traj_wp[1], sim_param.round_value+2)
            orig_traj_z = round(traj_wp[2], sim_param.round_value+2)
                        
            ## eef pos
            tmp_traj += [orig_traj_x, orig_traj_y, orig_traj_z]
            
            ## eef_orien                         
            tmp_traj += zero_vector
            
            ## obj pos and orientation
            for obj_id in range(len(obj_name_vector)):
                if obj_id==0:
                    tmp_traj += obj_pos_vector[0]
                    tmp_traj += zero_vector
                else:
                    tmp_traj += obj_pos_vector[1]
                    tmp_traj += [2,2,2]

                if exp_iros_one_obj:
                    ## fake cylinder_pos based on cube pos
#                    fake_pos = copy.copy(obj_pos_vector[0])
#                
#                    ## 2 areas created, not intersecting the trajectory space                                    
#                    ## chose area
#                    rand = random.randint(0,1)
#                    if rand: ## area close to the robot
#                        fake_pos[0] += random.uniform(0.1,0.2)
#                    else:
#                        fake_pos[0] -= random.uniform(0.1,0.2)                    
#                    fake_pos[1] += random.uniform(-0.3,0.3)
#                    fake_pos[2] = -0.075
#                    fake_pos = [round(value, sim_param.round_value) for value in fake_pos]
                        
                    fake_pos = []
                    fake_pos.append(obj_pos_vector[0][0] + 0.2)
                    fake_pos.append(obj_pos_vector[0][1] + 0.2)
                    fake_pos.append(-0.075)

                    tmp_traj += fake_pos
                    tmp_traj += [2,2,2]
                    
            
        ## add displacement for final obj pos
        tmp_traj.append(traj[-1][0])
        tmp_traj.append(traj[-1][1])
        tmp_traj.append(traj[-1][2])
        
        tmp_traj += zero_vector
        displ_vector = [0,0,0]
        if effect == 'right':
            displ_vector[1] -= sim_param.obj_displacement
        elif effect == 'left':
            displ_vector[1] += sim_param.obj_displacement
        elif effect == 'close':
            displ_vector[0] -= sim_param.obj_displacement            
        elif effect == 'far':
            displ_vector[0] += sim_param.obj_displacement
        tmp_obj_pos = [x+y for x,y in zip(obj_pos_vector[0], displ_vector)]
        tmp_traj += tmp_obj_pos
        tmp_traj += zero_vector        
        if exp_iros_two_obj:
            tmp_traj += obj_pos_vector[1]
            tmp_traj += [2,2,2]
        if exp_iros_one_obj:
            tmp_traj += fake_pos
            tmp_traj += [2,2,2]
            
        return [tmp_traj]       

    
def generate_dataset(effect):
        
    orig_eef_pos, obj_vector = get_environment_pos()
        
    traj_vector = []
    traj_diverse_vector = []
    for i in range(nb_init_traj):
        eef_pos = copy.copy(orig_eef_pos)
        
        if exp_two_push:
            cube_pos = obj_vector[0]
            cylinder_pos = obj_vector[1]
            
            ## all trajs converge into a mid pos
            mid_pos = copy.copy(cube_pos)
            mid_pos[1] += 0.1
            mid_pos[2] += 0.02
            
            ''' Create new inital position '''
            if i != 0:
                eef_pos_range = sim_param.new_obj_pos_dist
                eef_pos[0] = eef_pos[0] + random.uniform(-eef_pos_range,eef_pos_range*2)    
                eef_pos[1] = eef_pos[1] + random.uniform(-eef_pos_range*2,eef_pos_range*2)
                eef_pos[2] = eef_pos[2] + random.uniform(-eef_pos_range,eef_pos_range)
                eef_pos = [round(pos, sim_param.round_value) for pos in eef_pos]        
        
            ''' Create main traj '''
            noise = wp_change/2
            if eef_pos[0] < cube_pos[0]:
                mid_vax_x = mid_pos[0] + random.uniform(-noise,0)
            else:
                mid_vax_x = mid_pos[0] + random.uniform(0,noise)
            mid_vax_y = mid_pos[1] + random.uniform(-noise,noise)
            mid_vax_z = mid_pos[2] + random.uniform(0,noise*2)
            
            var_x_vector = (linspace(eef_pos[0], mid_vax_x,
                            int(nb_steps/2))).tolist()
            var_x_vector = [round(pos, sim_param.round_value+2) for pos in var_x_vector]
            var_x_vector_tmp = [mid_vax_x] + [mid_pos[0] for i in range(len(var_x_vector),nb_steps)][1:]    
            var_x_vector += var_x_vector_tmp
            
            if push_cylinder:
                tmp_obj_pos = cylinder_pos
            else:
                tmp_obj_pos = cube_pos
            var_y_vector = (linspace(eef_pos[1], mid_vax_y,
                            int(nb_steps/2))).tolist()
            var_y_vector = [round(pos, sim_param.round_value+2) for pos in var_y_vector]
            var_y_vector_tmp = [mid_vax_y] + (linspace(mid_pos[1], tmp_obj_pos[1], int(nb_steps/2))).tolist()[1:]
            var_y_vector += var_y_vector_tmp
            
            var_z_vector = (linspace(eef_pos[2], mid_vax_z,
                            int(nb_steps/2))).tolist()
            var_z_vector = [round(pos, sim_param.round_value+2) for pos in var_z_vector]
            var_z_vector_tmp = [mid_vax_z] + [mid_pos[2] for i in range(len(var_z_vector),nb_steps)][1:]
            var_z_vector += var_z_vector_tmp
            
            traj = [[var_x_vector[i], var_y_vector[i], var_z_vector[i]] 
                     for i in range(len(var_x_vector))]
                         
            ## to touch cylinder after pushing the box
            if push_cylinder:
                nb_final_steps = int(nb_steps*0.3)
                
                final_traj_sections_x = (linspace(traj[-1][0], cylinder_pos[0],
                                int(nb_final_steps+1))).tolist()
                final_traj_sections_x = [round(pos, sim_param.round_value+2) for pos in final_traj_sections_x]
                final_traj_sections_y = nb_final_steps * [traj[-1][1]]
                final_traj_sections_z = nb_final_steps * [traj[-1][2]]
                for i in range(nb_final_steps):
                    traj.append([final_traj_sections_x[i+1],
                                 final_traj_sections_y[i],
                                 final_traj_sections_z[i]])
            traj_vector.append(traj)
        
        elif exp_iros_one_obj or exp_iros_two_obj:

            cube_pos = obj_vector[0]
            if exp_iros_two_obj:
                cylinder_pos = obj_vector[1]

            ''' Modify initial position '''            
            if i != 0:
                eef_pos_range = sim_param.new_obj_pos_dist
                eef_pos[0] = eef_pos[0] + random.uniform(-eef_pos_range/2,eef_pos_range/2)    
                eef_pos[1] = eef_pos[1] + random.uniform(-eef_pos_range/2,eef_pos_range/2)
                eef_pos[2] = eef_pos[2] + random.uniform(-eef_pos_range/2,eef_pos_range/2)
                eef_pos = [round(pos, sim_param.round_value) for pos in eef_pos]              

            ''' Create main traj '''                   
            ## create noise and contact z
            noise = wp_change/2
            z_contact = round(cube_pos[2] - cube_height/2, 2)

            main_traj = [eef_pos]

            if exp_iros_one_obj:
                main_traj.append([cube_pos[0] + random.uniform(-noise,noise), ## all trajs converge into a mid pos
                                  cube_pos[1] + 0.30 + random.uniform(-noise,noise*3),
                                  z_contact + random.uniform(-noise,noise)])    
                                  
                main_traj.append([cube_pos[0] + random.uniform(-noise/1,noise/1), 
                                  cube_pos[1] - 0.05 + random.uniform(-noise/1,noise/1),
                                  z_contact + random.uniform(-noise/1,noise/1)])                                  
                
            else:
                            
                main_traj.append([cube_pos[0] + random.uniform(-noise,noise), ## all trajs converge into a mid pos
                                  cube_pos[1] + 0.30 + random.uniform(-noise,noise*3),
                                  z_contact + random.uniform(-noise,noise)])
    
                main_traj.append([cube_pos[0] + random.uniform(-noise,noise), ## change direction
                                  cube_pos[1] + 0.25 + random.uniform(-noise,noise),
                                  z_contact + random.uniform(-noise,noise)])
                                  
                main_traj.append([cube_pos[0] - 0.10 + random.uniform(-noise,noise), ## close
                                  cube_pos[1] + 0.15 + random.uniform(-noise,noise),
                                  z_contact + random.uniform(-noise,noise)])
                                  
                main_traj.append([cube_pos[0] + random.uniform(-noise/1,noise/1), ## center cylinder
                                  cube_pos[1] + 0.15 + random.uniform(-noise/1,noise/1), ## y = 0
                                  z_contact + random.uniform(-noise/1,noise/1)])
    
                main_traj.append([cube_pos[0] + random.uniform(-noise/1,noise/1), 
                                  cube_pos[1] - 0.05 + random.uniform(-noise/1,noise/1),
                                  z_contact + random.uniform(-noise/1,noise/1)])

            ''' Split each traj on small segments '''
            var_x_vector = []
            var_y_vector = []
            var_z_vector = []
            nb_segments = 4
            for pos in range(len(main_traj)-1):
                
                if exp_iros_one_obj:
                    if pos == 0:
                        tmp_nb_segments = nb_segments * 2
                    elif pos == 1:
                        tmp_nb_segments = nb_segments * 4
                else:                
                    if pos == 0:
                        tmp_nb_segments = nb_segments * 2
                    elif pos == 1:
                        tmp_nb_segments = 3
                    elif pos == 4:
                        tmp_nb_segments = nb_segments * 2
                    else:
                        tmp_nb_segments = nb_segments
                
                
                if main_traj[pos][0] != main_traj[pos+1][0]: ## X
                    tmp_sector_x = (linspace(main_traj[pos][0],
                                          main_traj[pos+1][0],
                                          tmp_nb_segments)).tolist()
                else:
                    tmp_sector_x = [main_traj[pos][0] for val in range(tmp_nb_segments)]
                var_x_vector += tmp_sector_x
                
                if main_traj[pos][1] != main_traj[pos+1][1]: ## Y
                    tmp_sector_y = (linspace(main_traj[pos][1],
                                          main_traj[pos+1][1],
                                          tmp_nb_segments)).tolist()
                else:
                    tmp_sector_y = [main_traj[pos][1] for val in range(tmp_nb_segments)]
                var_y_vector += tmp_sector_y;
                
                if main_traj[pos][2] != main_traj[pos+1][2]: ## Z                   
                    tmp_sector_z = (linspace(main_traj[pos][2],
                                          main_traj[pos+1][2],
                                          tmp_nb_segments)).tolist()
                else:
                    tmp_sector_z = [main_traj[pos][2] for val in range(tmp_nb_segments)]
                var_z_vector += tmp_sector_z;                          

            traj = [[var_x_vector[i], var_y_vector[i], var_z_vector[i]] 
                     for i in range(len(var_x_vector))]

            traj_vector.append(traj)        
        
        ''' Create diverse trajs '''
        obj_vector = [cube_pos]
        if exp_iros_two_obj:
            obj_vector.append(cylinder_pos)
        tmp_div_traj_vector = create_diverse_trajs(traj, 
                                                   obj_vector,
                                                   effect)
        traj_diverse_vector += tmp_div_traj_vector
    
    ''' Plot traj '''
    ax = plot_setup(obj_vector)
    plot_traj(ax, traj_vector)
        
    plt.show()

    return traj_diverse_vector

''' 
Write the trajs dataset 
'''     
def write_dataset(traj_vector):    
    file = open(filename, 'w')
    for traj in traj_vector:
        for value in traj:
            file.write(str(value))
            file.write(',')
        file.write('\n')     

''' 
Read the trajs dataset 
'''     
def read_dataset(filename):    
    lines = open(filename, 'r').readlines()    
    traj_vector = []
    obj_vector = []
    for line in lines:
        pos_rot_vector = line[:-2].split(',') ## remove final , and EOL
        traj = []
        obj = []
        for pos in range(0, len(pos_rot_vector), block_of_info):
            current_x = float(pos_rot_vector[pos+0])
            current_y = float(pos_rot_vector[pos+1])
            current_z = float(pos_rot_vector[pos+2])
            traj.append([current_x,
                         current_y,
                         current_z])
            for curr_obj_id in range(1,nb_objects+1):
                obj_pos_rot = pos_rot_vector[pos+6*curr_obj_id : pos+6*curr_obj_id + 6]
                obj_pos_rot = [float(value) for value in obj_pos_rot]
                obj.append(obj_pos_rot)
                
        traj_vector.append(traj)
        obj_vector.append(obj)
    
    return traj_vector, obj_vector
        
if __name__ == "__main__":

    exp_two_push = False
    exp_iros_one_obj = False
    exp_iros_two_obj = True
    
    cube_height = 0.08
    cylinder_height = 0.09

    if sim_param.real_robot:
        filename = '/home/maestre/.ros/eef_trajectory_recorder.csv'    
    else:
        filename = '../../../../a2l_exp_baxter_actions/src/generated_datasets/directed_dataset.csv'    

    obj_name_vector = ['cube']
    if exp_two_push or exp_iros_two_obj:
        obj_name_vector.append('cylinder')
            
    push_cylinder = len(obj_name_vector) > 1            
    nb_objects = len(obj_name_vector)
    block_of_info = 6 + 6*nb_objects
    
    create = True
    
    if create: ## create
        print('GENERATING DATASET')
        nb_diverse_trajs = 100
        wp_change = 0.03
        round_value = 2
        
        nb_init_traj = nb_diverse_trajs
        nb_steps = 18 ## even number
             
        if not sim_param.real_robot:
            success = ros_services.call_restart_world("all")
            if not success:
                print("ERROR - restart_world failed")
        
        traj_vector = generate_dataset('right')
        res = write_dataset(traj_vector)
        
    else: ## plot
        print('VISUALIZING DATASET')
        traj_vector, obj_vector = read_dataset(filename)
        ax = plot_setup(obj_vector)
        color_vector = plot_traj(ax, traj_vector)
        plot_objects(ax, obj_vector)
        plot_object_change(ax, obj_vector, color_vector)
        
    