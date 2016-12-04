# -*- coding: utf-8 -*-
"""
Created on Sat Mar 26 15:47:03 2016

@author: maestre
"""

#!/usr/bin/env python
# a bar plot with errorbars
import numpy as np
import matplotlib.pyplot as plt

N = 3
ind = np.arange(N)  # the x locations for the groups
width = 0.2       # the width of the bars
fig, ax = plt.subplots()

## success
success = (20, 35, 30)
#menStd = (2, 3, 4, 1, 2)
rects1 = ax.bar(ind, success, width, color='g') ##, yerr=menStd)

## false_positive
false_positive = (25, 32, 34)
#womenStd = (3, 5, 2, 3, 3)
rects2 = ax.bar(ind + width, false_positive, width, color='b') #, yerr=womenStd)

## fail
fail = (25, 32, 34)
#womenStd = (3, 5, 2, 3, 3)
rects3 = ax.bar(ind + 2*width, fail, width, color='r') #, yerr=womenStd)

# add some text for labels, title and axes ticks
ax.set_ylabel('Scores')
ax.set_title('DIRECTED dataset - Scores by learning algorithm')
ax.set_xticks(ind + width)
ax.set_xticklabels(('Hand-coded', 'Hill-climbing', 'Tabu-search'))
ax.set_xlim(0,2.6)
ax.set_ylim(0,50)

ax.legend((rects1[0], rects2[0], rects3[0]), ('Success', 'False positive', 'Fail'))


#def autolabel(rects):
#    # attach some text labels
#    for rect in rects:
#        height = rect.get_height()
#        ax.text(rect.get_x() + rect.get_width()/2., 1.05*height,
#                '%d' % int(height),
#                ha='center', va='bottom')
#
#autolabel(rects1)
#autolabel(rects2)

plt.show()