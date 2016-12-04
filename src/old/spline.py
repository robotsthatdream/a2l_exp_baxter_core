# -*- coding: utf-8 -*-
"""
Created on Fri Mar  4 10:17:38 2016

@author: maestre
"""

import numpy as np
import matplotlib.pyplot as plt
import math

points = np.array([(1,0), (1.5, 1), (0, 0)])
# get x and y vectors
x = points[:,0]
y = points[:,1]

## calculate polynomial
#z = np.polyfit(x, y, 3)
#f = np.poly1d(z)
#
## calculate new x's and y's
#x_new = np.linspace(x[0], x[-1], 50)
#y_new = f(x_new)
#
#plt.plot(x,y,'o', x_new, y_new)
#plt.xlim([x[0]-1, x[-1] + 1 ])
#plt.show()

from scipy import interpolate
tck,u = interpolate.splprep([x,y],k = 2)
x_i,y_i = interpolate.splev(np.linspace(0,1,20),tck)
plt.plot(x_i, y_i, '-', c='b')
plt.plot(x_i, y_i, 'o', c='r')