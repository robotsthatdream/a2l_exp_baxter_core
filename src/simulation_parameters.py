# -*- coding: utf-8 -*-
"""
@author: maestre
"""

import math

''' Main parameters '''
nb_min_init_pos = 4 ## better if mod 4
nb_min_orientation_sections = 8
nb_min_inclination_sections = 4
nb_min_distance_sections = 3
exec_traj = False

semi_random_trajs = True ## initial trajs using dicretized movs
discr_hand_coded = False
discr_random = False


''' Global variables '''
print_time = False ## print elapsed time for some functions
plot_dataset_stats = True ## plot the content of the discretized dataset
new_dataset = False ## True to generate a new dataset, False to use current one
dpi = 80 
round_value = 2

print_prob_table_values = False ## print the CPTs values
print_unknown_affordances = False ## print name of unknown affordances

''' Folder paths '''
generated_files_folder = 'generated_files/'
results_folder = 'results/'
exp_dataset_size_folder = 'dataset_size/'
exp_discr_folder = 'discretization/'
plots_folder = 'plots/'

''' Inferred trajs '''
plot_trajs = False ## plot ALL inferred trajectory
plot_some_trajs = False ## plot SOME inferred trajectory
save_trajs = False ## save ALL inferred trajectory
save_some_trajs = False ## save SOME inferred trajectory

''' Variables '''
distance_param = True

''' Scores '''
score_likelihood = False
score_bic = True
score_aic = False
score_k2 = False

''' Stats '''
plot_stats = True ## plot the resulting stats
save_stats = False ## save the resulting plot stats
print_stats = False ## print the numeric stats vectors

''' Print datasets '''
print_directed_dataset_extended = False
print_discr_random_dataset = False

''' Tests '''
test_directed_dataset_extended = False
test_statistics = False

''' Debug '''
debug = False
debug_infer = True
debug_services = True

''' Experiment discretization selection'''
#experiment_type = 'discretization_selection'
orientation_discr_vector_size = 4
distance_discr_vector_size = 3

''' Experiment A2L '''
experiment_type = 'a2l_dataset_extension'
plot_dataset_size_score = True ## plot the score in the dataset size plots
nb_dataset_sizes = 1 ## number of increments of init pos : 8->16->32->64
eef_z_value = 0.14
obj_name = "cube"

''' ROS execution '''
feedback_topic = '/a2l/traj_exec_info'
feedback_window = 2
current_feedback_window = 0
obj_moved = False
obj_moved_threshold = 0.1
inferred_traj = []

''' Experiment set-up values '''
#obj_pos = [0,0,0]
obj_side = 0.1
obj_displacement = 0.3
circumference_radio = 0.2
colormap_values = ['red','yellow','green','black']

''' Experiment run values '''
#nb_runs = 1
fixed_obj_pos = True ## if false random obj pos during validation phase
#obj_displacement = obj_side
same_move_value = 0.025 
#same_orientation_value = obj_side/2 ## ej. with same x value, y=0 is under 0.1, 'down'
step_length = 0.05 # obj_side/2
random_max_movs = 7
inferred_max_moves = 40
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
effect_values = ['left', 'right', 'close', 'far']

remote_far_boundary_value = 2 * obj_side
far_close_boundary_value = 1 * obj_side
orien_min_angle = -math.pi + math.pi/2
orien_max_angle = math.pi + math.pi/2
inclin_min_angle = 0 # -math.pi/2
inclin_max_angle = math.pi #/2

''' Performance computation '''
perf_success_value = 4
perf_f_p_value = 1
perf_fail_value = 0

''' Extend dataset '''
nb_adapted_iterations = 50
extend_max_trajs = 50
extend_max_movs = random_max_movs/2
nb_init_pos_for_adaption = 4
score_threshold = 0
only_store_different_effects = False ## if TRUE only store new effects obtained
                                    ## based on a succesfull traj (better diversity)
max_repeated_perf_value = 1500 ## nb iterations until generating new trajs for init_pos and effect
nb_desired_effect_reproduced = 20 ## nb new effect-related trajs generated

