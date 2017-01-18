# -*- coding: utf-8 -*-
"""
Created on Mon Mar  7 15:37:47 2016

@author: maestre
"""

import os, sys
run_path = os.path.abspath(os.path.join('..', '..'))
sys.path.append(run_path)
import simulation_parameters as sim_param

class Point:
    def __init__ (self, _x, _y, _z):
        self.x = _x
        self.y = _y
        self.z = _z        
    
    def x(self):
        return self.x
        
    def y(self):
        return self.y        
        
    def z(self):
        return self.z

 
''' Given three colinear points p, q, r, the function checks if 
    point q lies on line segment 'pr' '''
def onSegment(p, q, r):
    if q.x <= max(p.x, r.x) and q.x >= min(p.x, r.x) and \
        q.y <= max(p.y, r.y) and q.y >= min(p.y, r.y):
       return True
 
    return False

''' 
To find orientation of ordered triplet (p, q, r).
The function returns following values
0 --> p, q and r are colinear
1 --> Clockwise
2 --> Counterclockwise
'''
def orientation(p, q, r):
#    // See http://www.geeksforgeeks.org/orientation-3-ordered-points/
#    // for details of below formula.
    val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
 
    if (val == 0):
        return 0 
 
    if (val > 0):
        return 1
    else:
        return 2

'''
The main function that returns true if line segment 'p1q1'
and 'p2q2' intersect.
'''
def doIntersect(p1,q1,p2,q2):
#    // Find the four orientations needed for general and
#    // special cases
    o1 = orientation(p1, q1, p2);
    o2 = orientation(p1, q1, q2);
    o3 = orientation(p2, q2, p1);
    o4 = orientation(p2, q2, q1);
 
#    // General case
    if (o1 != o2 and o3 != o4):
        return True
 
#    // Special Cases
#    // p1, q1 and p2 are colinear and p2 lies on segment p1q1
    if (o1 == 0 and onSegment(p1, p2, q1)):
        return True
 
#    // p1, q1 and p2 are colinear and q2 lies on segment p1q1
    if (o2 == 0 and onSegment(p1, q2, q1)):
        return True
 
#    // p2, q2 and p1 are colinear and p1 lies on segment p2q2
    if (o3 == 0 and onSegment(p2, p1, q2)):
        return True
 
#     // p2, q2 and q1 are colinear and q1 lies on segment p2q2
    if (o4 == 0 and onSegment(p2, q1, q2)):
        return True
 
    return False
 
## Driver program to test above functions
#p1 = Point(1, 1)
#q1 = Point(10, 1)
#p2 = Point(1, 2)
#q2 = Point(10, 2)
# 
#print(doIntersect(p1, q1, p2, q2))
# 
#p1 = Point(10, 0)
#q1 = Point(0, 10)
#p2 = Point(0, 0)
#q2 = Point(10, 10)
#print(doIntersect(p1, q1, p2, q2))
# 
#p1 = Point(-5, -5)
#q1 = Point(0, 0)
#p2 = Point(1, 1)
#q2 = Point(10, 10)
#print(doIntersect(p1, q1, p2, q2))

# if interaction from the top -> obj_pos moves down (0, -.3, 0)
# if interaction from the bottom -> obj_pos moves up (0, .3, 0)
# if interaction from the left -> obj_pos moves right (0.3, 0, 0)
# if interaction from the right -> obj_pos moves left (-0.3, 0, 0)        

def check_obj_moved(box_pos,
                    _wp, 
                    _wp_next):
                        
#    box_pos = sim_param.obj_pos
    box_side = sim_param.obj_side # * 2   
    
    ## object corners
    l_bottom_front_right = Point(box_pos[0] + box_side/2, 
                                 box_pos[1] - box_side/2, 
                                 box_pos[2] - box_side/2)
#    print('l_bottom_front_right', l_bottom_front_right.x,
#                                  l_bottom_front_right.y,
#                                  l_bottom_front_right.z,)
    
    l_bottom_front_left = Point(box_pos[0] + box_side/2, 
                                 box_pos[1] + box_side/2, 
                                 box_pos[2] - box_side/2)                                 
    l_bottom_back_right = Point(box_pos[0] - box_side/2, 
                                 box_pos[1] - box_side/2, 
                                 box_pos[2] - box_side/2)
    l_bottom_back_left = Point(box_pos[0] - box_side/2, 
                                 box_pos[1] + box_side/2, 
                                 box_pos[2] - box_side/2)                                 

    l_top_front_right = Point(box_pos[0] + box_side/2, 
                                 box_pos[1] - box_side/2, 
                                 box_pos[2] + box_side/2)
    l_top_front_left = Point(box_pos[0] + box_side/2, 
                                 box_pos[1] + box_side/2, 
                                 box_pos[2] + box_side/2)                                 
    l_top_back_right = Point(box_pos[0] - box_side/2, 
                                 box_pos[1] - box_side/2, 
                                 box_pos[2] + box_side/2)
    l_top_back_left = Point(box_pos[0] - box_side/2, 
                                 box_pos[1] + box_side/2, 
                                 box_pos[2] + box_side/2)
    
    obj_moved = False
    i = 0
    while i < 2 and not obj_moved:
        wp_mid_point_x = (_wp[0] + _wp_next[0]) / 2
        wp_mid_point_y = (_wp[1] + _wp_next[1]) / 2
        wp_mid_point_z = (_wp[2] + _wp_next[2]) / 2
        if i == 0:
            wp = Point(_wp[0], _wp[1], _wp[2])        
            wp_next = Point(wp_mid_point_x, wp_mid_point_y, wp_mid_point_z)
        else:
            wp = Point(wp_mid_point_x, wp_mid_point_y, wp_mid_point_z)
            wp_next = Point(_wp_next[0], _wp_next[1], _wp_next[2])
#        print('EEF TMP POS', wp_next.x, wp_next.y, wp_next.z)

#        if doIntersect(wp, wp_next, l_bottom_front_right, l_bottom_front_left) or \
#           doIntersect(wp, wp_next, l_bottom_front_right, l_bottom_back_right) or \
#           doIntersect(wp, wp_next, l_bottom_front_right, l_top_front_right) or \
#           doIntersect(wp, wp_next, l_bottom_front_left, l_bottom_back_left) or \
#           doIntersect(wp, wp_next, l_bottom_front_right, l_bottom_front_left) or \
#           doIntersect(wp, wp_next, l_bottom_front_right, l_bottom_front_left) or \
#           doIntersect(wp, wp_next, l_bottom_front_right, l_bottom_front_left) or \
#           doIntersect(wp, wp_next, l_bottom_front_right, l_bottom_front_left):
#           obj_moved = True

        if wp_next.x <= l_bottom_front_right.x and \
            wp_next.y >= l_bottom_front_right.y and \
            wp_next.z >= l_bottom_front_right.z and \
            wp_next.x <= l_bottom_front_left.x and \
            wp_next.y <= l_bottom_front_left.y and \
            wp_next.z >= l_bottom_front_left.z and \
            wp_next.x >= l_bottom_back_right.x and \
            wp_next.y >= l_bottom_back_right.y and \
            wp_next.z >= l_bottom_back_right.z and \
            wp_next.x >= l_bottom_back_left.x and \
            wp_next.y <= l_bottom_back_left.y and \
            wp_next.z >= l_bottom_back_left.z and \
            wp_next.x <= l_top_front_right.x and \
            wp_next.y >= l_top_front_right.y and \
            wp_next.z <= l_top_front_right.z and \
            wp_next.x <= l_top_front_left.x and \
            wp_next.y <= l_top_front_left.y and \
            wp_next.z <= l_top_front_left.z and \
            wp_next.x >= l_top_back_right.x and \
            wp_next.y >= l_top_back_right.y and \
            wp_next.z <= l_top_back_right.z and \
            wp_next.x >= l_top_back_left.x and \
            wp_next.y <= l_top_back_left.y and \
            wp_next.z <= l_top_back_left.z:
                obj_moved = True
        i += 1
    return obj_moved
    
    

#'''
#Given a effect return the related final position
#'''
#def identify_effect_final_pos(_obj_pos, _effect):
#    if _effect == 'right':
#        return [_obj_pos[0] + sim_param.obj_displacement, _obj_pos[1], 0]
#    elif _effect == 'left':
#        return [_obj_pos[0] - sim_param.obj_displacement, _obj_pos[1], 0]
#    elif _effect == 'up':
#        return [_obj_pos[0], _obj_pos[1] + sim_param.obj_displacement, 0]
#    elif _effect == 'down':
#        return [_obj_pos[0], _obj_pos[1] - sim_param.obj_displacement, 0] 
#        
#'''
#Given a final position identify the effect
#'''
#def identify_effect(_obj_pos):
#    if _obj_pos == \
#        [sim_param.obj_pos[0] + sim_param.obj_displacement, 
#         sim_param.obj_pos[1], 0]:
#        return 'right'        
#    elif _obj_pos == \
#        [sim_param.obj_pos[0] - sim_param.obj_displacement, 
#         sim_param.obj_pos[1], 0]:
#        return 'left'
#    elif _obj_pos == \
#        [sim_param.obj_pos[0], 
#         sim_param.obj_pos[1] + sim_param.obj_displacement, 0]:
#        return 'up'        
#    elif _obj_pos == \
#        [sim_param.obj_pos[0], 
#         sim_param.obj_pos[1] - sim_param.obj_displacement, 0]:
#        return 'down'
#    else:
#        print('ERROR - identify_effect : no effect identified')
        
'''
Given a final position identify the effect
'''
def identify_effect_3d(initial_obj_pos, final_obj_pos):
    delta_x = abs(final_obj_pos[0] - initial_obj_pos[0])
    delta_y = abs(final_obj_pos[1] - initial_obj_pos[1])
    
    if delta_x > delta_y: ## up or down
        if final_obj_pos[0] > initial_obj_pos[0] :
            return 'far'
        elif final_obj_pos[0] < initial_obj_pos[0] :    
            return 'close'
    else:
        if final_obj_pos[1] > initial_obj_pos[1] :
            return 'left'
        elif final_obj_pos[1] < initial_obj_pos[1] :    
            return 'right'
   
'''
Test
'''
if __name__ == '__main__':
#    print(identify_effect([-0.3,0,0]))
#    print(identify_effect_3d([0,0,0], [-.2,-0.3,0]))

#    print(check_obj_moved([0,0,0], [0,0.5,0],[0,0.01,0])) # to the bottom
#    print(check_obj_moved([0,0,0], [0,-0.5,0],[0,-0.01,0])) # up
#    print(check_obj_moved([0,0,0], [0.5,0,0],[0.01,0,0])) # to the left
#    print(check_obj_moved([0,0,0], [-0.5,-0,0],[-0.01,0,0])) # to the right
#    
#    print(check_obj_moved([0,0,0], [-0.2,0,0],[0,-0.2,0])) # to the right
#    print(check_obj_moved([0,0,0], [0.2,0,0],[0,-0.2,0])) # to the left
#    print(check_obj_moved([0,0,0], [0.2,0,0],[0,1,0])) # to the left
#    print(check_obj_moved([0,0,0], [-0.1,-1,0],[0.1,10,0])) # up

    print(check_obj_moved([0.65, 0.1, -0.11], 
                          [0.7, 0.1, -0.11],[0.7, 0.15, -0.11]))