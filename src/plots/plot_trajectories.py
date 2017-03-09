from __future__ import print_function

import os, sys
lib_exp_path = os.path.realpath(os.path.abspath(
                os.path.join('..')))
sys.path.append(lib_exp_path)
import simulation_parameters as sim_param

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Rectangle
import mpl_toolkits.mplot3d.art3d as art3d
import numpy as np
from math import sin,cos,pi




class Trajectory():
    def __init__(self):
        self.obj_init_pos = []
        self.obj_final_pos = []
        self.eef_pos_vector = []
        
    def get_obj_init_pos(self):
        return self.obj_init_pos
    def set_obj_init_pos(self, x, y, z):
        self.obj_init_pos = [x, y, z]
        
    def get_obj_final_pos(self):
        return self.obj_final_pos
    def set_obj_final_pos(self, x, y, z):
        self.obj_final_pos = [x, y, z]        
        
    def add_eef_pos(self, x, y, z):
        self.eef_pos_vector.append([x,y,z])
    def get_eef_pos_vector(self):
        return self.eef_pos_vector
    def print_me(self):
        print('Trajectory values: ')
        print(self.obj_init_pos)
        print(self.obj_final_pos)
        print(self.eef_pos_vector)  

'''
a
'''
def get_trajs(filename):
    trajs_vector = []
    lines = open(filename, 'r').readlines()
    for line in lines:
        curr_traj = Trajectory()
        pos_rot_vector = line[:-2].split(',')
        curr_traj.set_obj_init_pos(float(pos_rot_vector[6]),
                                   float(pos_rot_vector[7]),
                                   float(pos_rot_vector[8]))
        curr_traj.set_obj_final_pos(float(pos_rot_vector[-6]),
                                    float(pos_rot_vector[-5]),
                                    float(pos_rot_vector[-4]))
#        print('Length wp_pos_rot_vector', len(pos_rot_vector))
#        print('Nb WPs', len(pos_rot_vector)//12)
        for pos in range(0, len(pos_rot_vector), 12):
            curr_traj.add_eef_pos(float(pos_rot_vector[pos+0]),
                                  float(pos_rot_vector[pos+1]),
                                  float(pos_rot_vector[pos+2]))
        trajs_vector.append(curr_traj)
#        print()
        curr_traj.print_me()
    return trajs_vector 

'''
a
'''
def gen_init_eef(radius, nb_init_pos, obj_pos):
    nb_init_pos += 2
    list_radians = [0]
    tmp = np.linspace(pi/(180.0/360), pi/180, nb_init_pos-1)
    tmp2 = tmp[:-1].tolist()
    tmp2.reverse()
    list_radians = list_radians + tmp2
    list_x_axis = []
    list_y_axis = []
    for a in list_radians[1:]:
        list_x_axis.append(obj_pos[0] + (cos(a))*radius)
        list_y_axis.append(obj_pos[1] + (sin(a))*radius)    
    return list_x_axis, list_y_axis, [0.14 for i in len(list_x_axis)]

#'''
#a
#'''
#def plot_trajs_2d(radius, 
#                  obj_pos,
#                  traj_vector):
#    
#    nb_traj = 0
#    while nb_traj < len(traj_vector): 
##    while nb_traj == 0:     
#        print('-------------> INIT POS', nb_traj//4)
#        for nb_traj_tmp in range(0,4):
#            print('TRAJECTORY', nb_traj + nb_traj_tmp)
#            curr_traj = traj_vector[nb_traj + nb_traj_tmp]
#            eef_pos_vector = curr_traj.get_eef_pos_vector()
#            eef_pos_vector_x = [t[0] for t in eef_pos_vector]
#            eef_pos_vector_y = [t[1] for t in eef_pos_vector]
#                    
#            # generate circle positions
#            list_x_axis, list_y_axis, list_z_axis = \
#                gen_init_eef(radius, nb_initial_pos, obj_pos)
#            
#            ''' Plot canvas ''' 
#            fig = plt.figure()
#            fig.set_size_inches(7,7)
#            ax = fig.add_axes([0, 0, 1, 1])
#            
#            # set axis limits
#            plt.xlim(obj_pos[0]-0.3, obj_pos[0]+0.3)
#            plt.ylim(obj_pos[1]-0.3, obj_pos[1]+0.3)
#         
#            # plot obj_init_pos
#            origin = Rectangle((obj_pos[0]-obj_side/2, 
#                                obj_pos[1]-obj_side/2), 
#                               obj_side, obj_side, fc="grey")
#            ax.add_patch(origin)
#             
#            # plot the big circle points
#            ax.plot(list_x_axis,list_y_axis,'o',c='r', markersize = 3)     
#            
#            # plot traj
#            if nb_traj == 0:
#                color = 'blue'
#            elif nb_traj == 1:
#                color = 'red'
#            elif nb_traj == 2:
#                color = 'yellow'
#            else:
#                color = 'green'                                
#            ax.plot(eef_pos_vector_x, eef_pos_vector_y, 
#                    '-',
#                    color = color)
#            ax.plot(eef_pos_vector_x, eef_pos_vector_y, 
#                    '*', 
#                    c='grey')
#            
#            plt.show()
#            plt.close()            
#        
#        nb_traj += 4
#        

'''
a
'''
def plot_trajs_3d(radius, 
                  obj_pos,
                  traj_vector):
   
    nb_traj = 0
    while nb_traj < len(traj_vector) :           
#        if nb_traj != 3:
#            nb_traj += 1
#            continue

#        if nb_traj != 0:
#            nb_traj += 1
#            continue
        
        print('-------------> INIT POS', nb_traj//4)
        # plot figure
        fig = plt.figure(figsize=(7,7))
        fig.clf()        
        fig.canvas.set_window_title(str(nb_traj))
        ax = Axes3D(fig)        
        for nb_traj_tmp in range(0,4):
#        for nb_traj_tmp in range(0,1):
            print('TRAJECTORY', nb_traj + nb_traj_tmp)
            curr_traj = traj_vector[nb_traj + nb_traj_tmp]
            eef_pos_vector = curr_traj.get_eef_pos_vector()
            eef_pos_vector_x = [float(t[0]) for t in eef_pos_vector]
            eef_pos_vector_y = [float(t[1]) for t in eef_pos_vector]
            eef_pos_vector_z = [float(t[2]) for t in eef_pos_vector]

#            # table        
#            ax.bar3d([.3], [-.8], [-.1-obj_side/2], 
#                     [.8], [1.6], [.0011], 
#                     color='brown', alpha=0.2,
#                     edgecolor = 'lightgrey')            
            
            # color for this trajectory
            if nb_traj_tmp == 0:
                traj_color = 'blue'
            elif nb_traj_tmp == 1:
                traj_color = 'red'
            elif nb_traj_tmp == 2:
                traj_color = 'goldenrod'
            else:
                traj_color = 'green'           
            
            # box init_pos
            ax.bar3d(obj_pos[0] - sim_param.cube_x/2, 
                     obj_pos[1] - sim_param.cube_y/2, 
                     obj_pos[2] - sim_param.cube_z/2, 
                     [sim_param.cube_y], [sim_param.cube_z], [sim_param.cube_z],
                     color='lightgrey',
                     alpha=0.2,
                     edgecolor='none')
                     
            # box final_pos
            obj_final_pos = curr_traj.get_obj_final_pos()
            ax.bar3d(obj_final_pos[0] - sim_param.cube_x/2, 
                     obj_final_pos[1] - sim_param.cube_y/2, 
                     obj_final_pos[2] - sim_param.cube_z/2, 
                     [sim_param.cube_y], [sim_param.cube_x], [sim_param.cube_z],
                     color=traj_color,
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

            # plot the big circle points
            ## TBD

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
            
            # plot traj
            ax.plot(eef_pos_vector_x, 
                    eef_pos_vector_y, 
                    eef_pos_vector_z,
                    '-',                                     
                    markersize=2,
                    color = traj_color)
            ax.plot(eef_pos_vector_x, 
                    eef_pos_vector_y, 
                    eef_pos_vector_z,
                    '*',                                     
                    markersize=3,
                    c='grey')
            
            plt.show()
#            plt.close()
        nb_traj += 4
            
   
   
'''
Main
'''
if __name__ == '__main__':
    nb_initial_pos = 4
    obj_pos = [0.65, 0.1, -0.145]
    radius = 0.2
    
    traj_vector = get_trajs("/home/maestre/git/a2l_exp_baxter_actions/src/generated_datasets/directed_dataset.csv")
    print('Nb trajs :', len(traj_vector))
#    plot_trajs_2d(radius, 
#               obj_pos,
#               traj_vector)
               
    plot_trajs_3d(radius, 
               obj_pos,
               traj_vector)
    
    
    
