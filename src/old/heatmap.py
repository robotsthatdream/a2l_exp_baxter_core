# _*_ coding: utf_8 _*_
"""
Created on Mon Jun 27 09:36:40 2016

@author: maestre
"""

from __future__ import print_function

from collections import Counter
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np


''' 
Orientations available for each effect
'''
class Orientation_values:
    def __init__(self):
        self.up = []        
        self.left_up = []
        self.left = [] 
        self.left_down = []
        self.down = []
        self.right_down = []
        self.right = []
        self.right_up = []        
    
    ''' Add new move value for a specific orientation '''
    def append(self, orientation, move):
        if orientation == 'up':
            self.up.append(move)
        elif orientation == 'left-up':
            self.left_up.append(move)
        elif orientation == 'left':
            self.left.append(move)
        elif orientation == 'left-down':
            self.left_down.append(move)
        elif orientation == 'down':
            self.down.append(move)
        elif orientation == 'right-down':
            self.right_down.append(move)
        elif orientation == 'right':
            self.right.append(move)
        elif orientation == 'right-up':
            self.right_up.append(move)
        else:
            print(orientation, move, 'not found')
    
    ''' Print moves available for each orientation '''
    def print_me(self):
        pos = 0
        for l in [self.up, self.left_up, self.left, self.left_down, 
                  self.down, self.right_down, self.right, self.right_up]:
            if pos == 0:
                current_list = 'up'
            elif pos == 1:               
                current_list = 'left-up'
            elif pos == 2:
                current_list = 'left'
            elif pos == 3:
                current_list = 'left-down'
            elif pos == 4:
                current_list = 'down'
            elif pos == 5:
                current_list = 'right-down'
            elif pos == 6:
                current_list = 'right'
            elif pos == 7:
                current_list = 'right-up'
            pos += 1
            
            print(current_list, Counter(l).items())
    
    ''' Get number of moves available for each orientation '''
    def get_move_repetitions(self, orientation):
        if orientation == 'up':
            current_orientation = self.up
        elif orientation == 'left-up':
            current_orientation = self.left_up
        elif orientation == 'left':
            current_orientation = self.left
        elif orientation == 'left-down':
            current_orientation = self.left_down
        elif orientation == 'down':
            current_orientation = self.down
        elif orientation == 'right-down':
            current_orientation = self.right_down
        elif orientation == 'right':
            current_orientation = self.right
        elif orientation == 'right-up':
            current_orientation = self.right_up
        else:
            print(orientation, 'not found')
            
        return Counter(current_orientation).items()
            
        
'''
Effects available in the experiment
'''
class Effect_values:
    def __init__(self):
        self.up = Orientation_values()
        self.left = Orientation_values()
        self.down = Orientation_values()
        self.right = Orientation_values()        
    
    ''' Add new (orientation,move) for a specific goal ''' 
    def append(self, effect, orien, move):
        if effect == 'up':
            self.up.append(orien, move)        
        elif effect == 'left':
            self.left.append(orien, move)
        elif effect == 'down':
            self.down.append(orien, move)
        elif effect == 'right':
            self.right.append(orien, move)
        else:
            print(effect, 'not found')

    ''' Print (orientation,move) for each effect '''
    def print_me(self):        
        pos = 0
        for g in [self.up, self.left, self.down, self.right]:
            if pos == 0:
                print ('\nUP\n')     
            if pos == 1:
                print ('\nLEFT\n')
            elif pos == 2:
                print ('\nDOWN\n')
            elif pos == 3:
                print ('\nRIGHT\n')           
            pos += 1

            g.print_me()            


    ''' Print a heatmap for each effect, showing the number of moves
        found for each (orientation,move)'''
    def print_stats(self):
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2,figsize=(13,8))
        
        ax_pos = 0
        for curr_effect in effect_values:
            if ax_pos == 0:
                ax1, heatmap1 = self.print_stats_effect(curr_effect, ax1)
            elif ax_pos == 1:
                ax2, heatmap2 = self.print_stats_effect(curr_effect, ax2)
            elif ax_pos == 2:
                ax3, heatmap3 = self.print_stats_effect(curr_effect, ax3)
            elif ax_pos == 3:
                ax4, heatmap4 = self.print_stats_effect(curr_effect, ax4)
            ax_pos += 1    
        
        fig.text(0.5, 0.04, 'MOVE', ha='center', va='center')
        fig.text(0.06, 0.5, 'ORIENTATION', ha='center', va='center', rotation='vertical')
        cax,kw = mpl.colorbar.make_axes([ax for ax in [ax1, ax2, ax3, ax4]])
        plt.colorbar(heatmap4, cax, kw)
        plt.show()
            
    '''Create previous heatmap '''
    def print_stats_effect(self, effect, ax):           
        ## Create matrix to fill
        stats_matrix = np.zeros(shape=(8,8))
        
        ## Select current effect list
        if effect == 'up':
            current_effect = self.up
        elif effect == 'left':
            current_effect = self.left 
        elif effect == 'down':
            current_effect = self.down
        elif effect == 'right':
            current_effect = self.right
        
        ## For each possible orientation print the available movements
        i = 0
        for current_orien in orientation_values:
            current_orien_move_vector = \
                current_effect.get_move_repetitions(current_orien)            

            ## get the available number of each movement
            j = -1
            nb_move = -1
            for m in current_orien_move_vector:
                nb_move = m[1]
                if m[0] == 'up':
                    j = 0
                elif m[0] == 'left-up':
                    j = 1
                elif m[0] == 'left':
                    j = 2
                elif m[0] == 'left-down':
                    j = 3
                elif m[0] == 'down':
                    j = 4
                elif m[0] == 'right-down':
                    j = 5
                elif m[0] == 'right':
                    j = 6
                elif m[0] == 'right-up':
                    j = 7
                stats_matrix [i,j] = nb_move
            i += 1
        
        ## Print stats
#        print ('\n',stats_matrix)
        heatmap = ax.pcolor(stats_matrix, 
                             cmap='PuBu', vmin=0, vmax=v_max)
        
        for y in range(stats_matrix.shape[0]):
            for x in range(stats_matrix.shape[1]):
                ax.text(x + 0.5, y + 0.5, '%.d' % stats_matrix[y,x],
                         horizontalalignment='center',
                         verticalalignment='center',
                         )
    
        # put the major ticks at the middle of each cell
        ax.set_xticks(np.arange(stats_matrix.shape[0])+0.5, minor=False)
        ax.set_yticks(np.arange(stats_matrix.shape[1])+0.5, minor=False)
        
        ax.invert_yaxis()               
        ax.set_title(effect.upper())
        
        row_labels = ['^', 'lu', '<', 'ld', 'v', 'rd', '>', 'ru']
        column_labels = ['^', 'lu', '<', 'ld', 'v', 'rd', '>', 'ru']
        ax.set_xticklabels(row_labels, minor=False)
        ax.set_yticklabels(column_labels, minor=False)
        
        return ax, heatmap


'''
Main
'''                        
effect_values = ['up', 'left', 'down', 'right']
orientation_values = ['up', 'left-up', 'left', 'left-down', 'down', 
                      'right-down', 'right', 'right-up']
directed_dataset = False

if directed_dataset:
    ''' DIRECTED '''   
    dataset = Effect_values()
    dataset_filename = '/home/maestre/dream/simple_dataset_exp/generated_files/directed_discr_wps.csv'
    lines = open(dataset_filename, 'r').readlines()
    v_max = len(lines)/10
    for line in lines[1:]:
        effect, orien, move = line[:-1].split(',')
        dataset.append(effect, orien, move)
    #dataset.print_me()
    dataset.print_stats()

else:
    ''' RANDOM '''
    dataset = Effect_values()
    dataset_filename = '/home/maestre/dream/simple_dataset_exp/generated_files/random_discr_wps.csv'
    lines = open(dataset_filename, 'r').readlines()
    v_max = len(lines)/10
    for line in lines[1:]:
        effect, orien, move = line[:-1].split(',')
        dataset.append(effect, orien, move)
    #dataset.print_me()
    dataset.print_stats()

