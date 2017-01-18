# -*- coding: utf-8 -*-
"""
@author: maestre
"""

from __future__ import print_function

import matplotlib.pyplot as plt
import numpy as np
from collections import OrderedDict
import operator
import matplotlib as mpl

import sys, os
run_path = os.path.abspath(os.path.join('..', '..'))
sys.path.append(run_path)
import simulation_parameters as sim_param

''' 
Orientations available for each effect
'''
class Orientation_dist_values:
    def __init__(self):
        self.orient_vector = \
            ['orien_'+str(x) for x in range(sim_param.nb_min_orientation_sections)]
        self.inclin_vector = \
            ['inclin_'+str(x) for x in range(sim_param.nb_min_inclination_sections)]
        self.dist_vector = \
            ['dist_'+str(x) for x in range(sim_param.nb_min_distance_sections)]
        self.values = self.create_values_dict()
    
    ''' Initializes the dictionary to store the values '''
    def create_values_dict(self):        
            
        values_dict = OrderedDict()
        for orien in self.orient_vector:
            for inclin in self.inclin_vector:
                for dist in self.dist_vector:
                    move_dict = OrderedDict()        
                    for m in sim_param.move_values:
                        move_dict[m] = 0                                
                    values_dict[orien, inclin, dist] = move_dict
        return values_dict
                
    ''' Add new move value for a specific (orien, dist) '''
    def add_value(self, orientation, inclin, move, dist):
        current_move_dict = self.values[orientation, inclin, dist]
        move_value = current_move_dict[move]
        current_move_dict[move] = move_value + 1
    
    ''' Print moves available for each orientation '''
    def print_me(self):    
        for orien in self.orient_vector:
            for inclin in self.inclin_vector:
                for dist in self.dist_vector:
                    print('\n' + str(orien), str(inclin), str(dist))
                    current_move_dict = self.values[orien, inclin, dist]
                    for tmp_d_key, tmp_d_value in current_move_dict.items():
                        print('', tmp_d_key, "-->", tmp_d_value)

    ''' Get best move for an (orient, dist) '''
    def get_move_repetitions(self, orien, inclin, dist):
        current_move_dict = self.values[orien, inclin, dist]
        best_move = \
            max(current_move_dict.iteritems(), key=operator.itemgetter(1))[0]
        best_move_value = current_move_dict[best_move]        
        return best_move, best_move_value
        
    ''' Get best move value for this effect '''
    def get_best_effect_move_value(self):        
        best_effect_move_value = 0
        for curr_orien in self.orient_vector:
            for curr_inclin in self.inclin_vector:
                for curr_dist in self.dist_vector:
                    tmp_max_move, tmp_max_move_value = \
                        self.get_move_repetitions(curr_orien,
                                                  curr_inclin,
                                                  curr_dist)
                    if tmp_max_move_value > best_effect_move_value:
                        best_effect_move_value = tmp_max_move_value
        return best_effect_move_value
'''
Effects available in the experiment
'''
class Effect_values:
    def __init__(self):
        self.far = Orientation_dist_values()
        self.left = Orientation_dist_values()
        self.close = Orientation_dist_values()
        self.right = Orientation_dist_values()        
    
    ''' Add new (orientation,move) for a specific goal ''' 
    def add_effect_value(self, effect, orien, inclin, move, dist=None):
        if effect == 'far':
            self.far.add_value(orien, inclin, move, dist)        
        elif effect == 'left':
            self.left.add_value(orien, inclin, move, dist)
        elif effect == 'close':
            self.close.add_value(orien, inclin, move, dist)
        elif effect == 'right':
            self.right.add_value(orien, inclin, move, dist)
        else:
            print('add_effect_value -', effect, 'not found')

    ''' Print (orientation,move) for each effect '''
    def print_me(self):        
        pos = 0
        for g in [self.far, self.left, self.close, self.right]:
            if pos == 0:
                print ('\nFAR\n')     
            if pos == 1:
                print ('\nLEFT\n')
            elif pos == 2:
                print ('\nCLOSE\n')
            elif pos == 3:
                print ('\nRIGHT\n')           
            pos += 1

            g.print_me()            

    ''' Get best move value for any (effect, orient, dist) '''
    def get_best_global_move_value(self):
        best_global_move_value = 0
        for curr_effect in [self.far, self.close, self.left, self.right]:
            curr_best_move = curr_effect.get_best_effect_move_value()
            if curr_best_move > best_global_move_value:
                best_global_move_value = curr_best_move
        return best_global_move_value

    ''' Plot a heatmap for each effect, showing the number of moves
        found for each (orientation, dist, move)'''
    def print_stats(self, v_max):
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2,figsize=(13,8))        
        v_max = self.get_best_global_move_value()
        
        ax_pos = 0
        for curr_effect in sim_param.effect_values:
            if ax_pos == 0:
                ax1, heatmap1 = self.print_stats_effect(curr_effect, ax1, v_max)
            elif ax_pos == 1:
                ax2, heatmap2 = self.print_stats_effect(curr_effect, ax2, v_max)
            elif ax_pos == 2:
                ax3, heatmap3 = self.print_stats_effect(curr_effect, ax3, v_max)
            elif ax_pos == 3:
                ax4, heatmap4 = self.print_stats_effect(curr_effect, ax4, v_max)
            ax_pos += 1    
        
        fig.text(0.5, 0.04, 'DISTANCE', ha='center', va='center')
        fig.text(0.06, 0.5, 'ORIENTATION', ha='center', va='center', rotation='vertical')
        cax,kw = mpl.colorbar.make_axes([ax for ax in [ax1, ax2, ax3, ax4]])
        plt.colorbar(heatmap4, cax, kw)
        cax.set_yticklabels(np.linspace(0, 100, 11, endpoint=True))
        plt.show()
    
    ''' Given a effect return a symbol '''
    def simplify_effect(self,effect):
        if effect == 'far':
            return 'f'
        elif effect == 'close':
            return 'c'
        elif effect == 'left':
            return 'l'
        elif effect == 'right':
            return 'r'
        elif effect == 'right-close':
            return 'r-c'
        elif effect == 'right-far':
            return 'r-f'
        elif effect == 'left-close':
            return 'l-c'
        elif effect == 'left-far':
            return 'l-f'
        else:
            print('ERROR - simplify_effect', effect, 'not known')
            sys.exit()
    
    '''Create previous heatmap '''
    def print_stats_effect(self, effect, ax, v_max):

        ## Create matrix to fill
        orient_vector = \
            ['orien_'+str(x) for x in range(sim_param.nb_min_orientation_sections)]
        inclin_vector = \
            ['inclin_'+str(x) for x in range(sim_param.nb_min_inclination_sections)]            
        dist_vector = \
            ['dist_'+str(x) for x in range(sim_param.nb_min_distance_sections)]
        tmp_array = []
        for i in range(len(orient_vector)):
            current_array = ["aaaaaaaaaaaaaaaaaaaaaaa" for x in range(len(dist_vector))]
            tmp_array.append(current_array)        
        stats_matrix_move = np.array(tmp_array)
            
        stats_matrix_move_value = np.zeros(shape=(len(orient_vector),
                                       len(dist_vector)))                                               
        ## Select current effect list
        if effect == 'far':
            current_effect = self.far
        elif effect == 'left':
            current_effect = self.left 
        elif effect == 'close':
            current_effect = self.close
        elif effect == 'right':
            current_effect = self.right 
        
        ## For each possible (orien, dist) plot the move with highest value
        for current_orien in range(len(orient_vector)):
            for current_inclin in range(len(inclin_vector)):
                for current_dist in range(len(dist_vector)):
                
                    best_move, best_move_value = \
                        current_effect.get_move_repetitions('orien_'+str(current_orien),
                                                            'inclin_'+str(current_inclin),
                                                            'dist_'+str(current_dist))
                    ## get the available number of each movement
                    stats_matrix_move [current_orien, current_dist] = \
                        best_move
                    stats_matrix_move_value [current_orien, current_dist] = \
                        (float(best_move_value)/v_max)*100                            
        
        ## heatmap
        heatmap = ax.pcolor(stats_matrix_move_value, 
                             cmap='Blues', 
                             linestyle=':', edgecolor='black',
                             vmin=0, vmax=100)
        
        for y in range(stats_matrix_move.shape[0]):
            for x in range(stats_matrix_move.shape[1]):
                if (stats_matrix_move_value[y,x]):
                    ax.text(x+0.5, y+0.5, 
                            self.simplify_effect(stats_matrix_move[y,x]),
                            horizontalalignment='center',
                            verticalalignment='center')

        ## plot as a table
        ax.invert_yaxis()  
    
        # put the major ticks at the middle of each cell
        ax.set_xticks(np.arange(stats_matrix_move_value.shape[1])+0.5, minor=False)
        ax.set_yticks(np.arange(stats_matrix_move_value.shape[0])+0.5, minor=False)        
        
#        row_labels = [x for x in range(len(orient_vector))]
#        column_labels = [x for x in range(len(dist_vector))]        
        row_labels = orient_vector
        column_labels = dist_vector        
        ax.set_xticklabels(column_labels, minor=False)
        ax.set_yticklabels(row_labels, minor=False)
        
        ax.set_title(effect.upper())
 
        return ax, heatmap
        
'''
Prepare to print the dataset stats
'''
def plot_dataset_stats(filename):
    dataset_values = Effect_values()                
    lines = open(filename, 'r').readlines()
    v_max = len(lines)/10
    for line in lines[1:]:
        if not sim_param.distance_param:
            effect, orien, inclin, move = line[:-1].split(',')            
            dataset_values.add_effect_value(effect, orien, inclin, move)
        else:
            effect, orien, inclin, move, dist = line[:-1].split(',')
            dataset_values.add_effect_value(effect, orien, inclin, move, dist)

#    dataset_values.print_me()
    dataset_values.print_stats(v_max)  

'''
Test
'''
if __name__ == '__main__':
    filename = '/home/maestre/git/a2l_exp_baxter_core/src/generated_files/directed_discr_wps.csv'
    plot_dataset_stats(filename)

