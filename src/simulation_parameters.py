# -*- coding: utf-8 -*-
"""
@author: maestre
"""

import math

''' Main parameters '''
real_robot = False
exec_traj = True
new_dataset = False ## False to use the initial current dataset

nb_min_init_pos = 1 ## better if mod 4
nb_min_orientation_sections = 16
nb_min_inclination_sections = 8
nb_min_distance_sections = 3
single_init_pos = False ## if False, circle of init pos around the box

semi_random_trajs = True ## initial and extended trajs using dicretized movs
discr_hand_coded = False
discr_random = False

''' Global variables '''
print_time = False ## print elapsed time for some functions
plot_dataset_stats = True ## plot the content of the discretized dataset
dpi = 300 
round_value = 3
effect_validation = 3 ## number of times a move in an axis must be > than 
                      ## in the other one to be accepted as effect

print_prob_table_values = False ## print the CPTs values
print_unknown_affordances = False ## print name of unknown affordances

''' Debug '''
debug = False
debug_infer = False
debug_services = False

''' Folder paths '''
generated_files_folder = 'generated_files/'
generated_datasets_folder = '/home/maestre/git/a2l_exp_baxter_actions/src/generated_datasets/'
results_folder = 'results/'
exp_dataset_size_folder = 'dataset_size/'
exp_discr_folder = 'discretization/'
plots_folder = 'plots/'

''' Inferred trajs '''
plot_trajs = False ## plot ALL inferred trajectory
plot_some_trajs = False ## plot SOME inferred trajectory
save_trajs = True ## save ALL inferred trajectory
save_some_trajs = False ## save SOME inferred trajectory

''' Variables '''
inclination_param = False

''' Scores '''
score_likelihood = False
score_bic = False
score_aic = True
score_k2 = False

''' Stats '''
plot_stats = True ## plot the resulting stats
save_stats = True ## save the resulting plot stats
print_stats = False ## print the numeric stats vectors

''' Print datasets '''
print_directed_dataset_extended = False
print_discr_random_dataset = False

''' Tests '''
test_directed_dataset_extended = False
test_statistics = False

''' Experiment discretization selection'''
#experiment_type = 'discretization_selection'
orientation_discr_vector_size = 4
distance_discr_vector_size = 3

''' Experiment A2L '''
experiment_type = 'a2l_dataset_extension'
plot_dataset_size_score = True ## plot the score in the dataset size plots
nb_dataset_sizes = 1 ## number of increments of init pos : 8->16->32->64
#eef_z_value = 0.14
eef_z_value = -0.145
eef_width = 0.02

''' Experiment Reproduce dataset (learn by demonstration) '''
#experiment_type = 'a2l_reproduce_dataset'
#nb_NN = 6 ## points around current position to infer next move
experiment_version = 'intermediate_state'
nb_infere_trajs = 1
max_inferred_traj_tries = 1

''' ROS execution '''
feedback_topic = '/a2l/traj_exec_info'
feedback_window = 1
current_feedback_window = 0
obj_moved = False
obj_moved_threshold = 0.001
inferred_traj = []

''' Experiment set-up values '''
obj_name_vector = ["cube"]#, 'cylinder']
dataset_nb_objects = 1
moved_obj_name_vector = ['cube'] # moved_obj_name_vector
#untucked_left_eef_pos = [0.58, 0.18, 0.11]
untucked_left_eef_pos = [0.85, 0.1, 0.14] ## to plot initial position
########### TODO RIGHTTTTTTTTTTTTTTTTTTTT INIT POS
#obj_side = 0.1
obj_displacement = 0.3
circumference_radio = 0.2
colormap_values = ['red','yellow','green','black']

''' Experiment run values '''
#nb_runs = 1
fixed_obj_pos = False ## if false random obj pos during validation phase
#obj_displacement = obj_side
#same_move_value = 0.015
#same_orientation_value = obj_side/2 ## ej. with same x value, y=0 is under 0.1, 'down'
step_length = 0.05 # obj_side/2 ## online computation based on dataset 
random_max_movs = 7
inferred_max_moves = 12
max_nb_executed_deltas = inferred_max_moves

''' Discretization predefined values'''
#orientation_values = ['up', 'left-up', 'left', 'left-down', 'down', 
#                      'right-down', 'right', 'right-up']
#distance_values = ['close', 'far', 'remote']

move_values = ['far', 
               'far_left', 'far_left_up', 'far_left_down',
               'far_right', 'far_right_up', 'far_right_down',
               'far_down', 'far_up',               
               'close', 
               'close_left', 'close_left_up', 'close_left_down',
               'close_right', 'close_right_up', 'close_right_down',
               'close_down', 'close_up',               
               'left', 'left_up', 'left_down',
               'right', 'right_up', 'right_down',
               'up',
               'down']               
effect_values = ['left', 'right', 
                 'close', 'close_left', 'close_right',
                 'far', 'far_left', 'far_right']
#effect_values = ['right']               

#remote_far_boundary_value = 2 * obj_side
#far_close_boundary_value = 1 * obj_side

orien_offset = round((2*math.pi/nb_min_orientation_sections)/2,  round_value)
#orien_min_angle = -math.pi + math.pi/2  + orien_offset
#orien_max_angle = math.pi + math.pi/2 + orien_offset
orien_min_angle = round(-math.pi + orien_offset, round_value)
orien_max_angle = round(math.pi + orien_offset, round_value)
inclin_min_angle = round(0, round_value)
inclin_max_angle = round(math.pi, round_value)

''' Performance computation '''
perf_success_value = 4
perf_f_p_value = 1
perf_fail_value = 0

''' Extend dataset '''
nb_adapted_iterations = 2
extend_max_trajs = 1
extend_max_movs = random_max_movs/1
nb_init_pos_for_adaption = nb_min_init_pos
score_threshold = 0
max_repeated_perf_value = 1500 ## nb iterations until generating new trajs for init_pos and effect
nb_desired_effect_reproduced = 20 ## nb new effect-related trajs generated

''' TO DO '''
#obj_side = 0.1
#box_side = obj_side
cube_x = 0.07
cube_y = 0.085
cube_z = 0.08
first_obj_pos = [0.65, 0.1, -0.145]
obj_too_far_distance = 0.1 ## to move it back close to the eef init pos
new_obj_pos_dist = 0.025 ## for new pos close to init obj pos 
radio = 0.2

