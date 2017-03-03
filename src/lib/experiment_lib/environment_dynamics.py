# -*- coding: utf-8 -*-
"""
Created on Mon Mar  7 15:37:47 2016

@author: maestre
"""

import os, sys
run_path = os.path.realpath(os.path.abspath(os.path.join('..', '..')))
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

#    box_side = sim_param.obj_side

    box_side_front = sim_param.cube_x ## side in front of the robot
    box_side_lateral = sim_param.cube_y
    box_side_top = sim_param.cube_z
        
    ## object corners
    l_bottom_front_right = Point(box_pos[0] + box_side_front/2,
                                 box_pos[1] - box_side_lateral/2, 
                                 box_pos[2] - box_side_top)    
    l_bottom_front_left = Point(box_pos[0] + box_side_lateral/2, 
                                 box_pos[1] + box_side_front/2, 
                                 box_pos[2] - box_side_top)                                 
    l_bottom_back_right = Point(box_pos[0] - box_side_lateral/2, 
                                 box_pos[1] - box_side_front/2, 
                                 box_pos[2] - box_side_top)
    l_bottom_back_left = Point(box_pos[0] - box_side_lateral/2, 
                                 box_pos[1] + box_side_front/2, 
                                 box_pos[2] - box_side_top)                                 

    l_top_front_right = Point(box_pos[0] + box_side_lateral/2, 
                                 box_pos[1] - box_side_front/2, 
                                 box_pos[2])
    l_top_front_left = Point(box_pos[0] + box_side_lateral/2, 
                                 box_pos[1] + box_side_front/2, 
                                 box_pos[2])                                 
    l_top_back_right = Point(box_pos[0] - box_side_lateral/2, 
                                 box_pos[1] - box_side_front/2, 
                                 box_pos[2])
    l_top_back_left = Point(box_pos[0] - box_side_lateral/2, 
                                 box_pos[1] + box_side_front/2, 
                                 box_pos[2])                                
    
    obj_moved = False
    i = 0
    while i < 2 and not obj_moved:
        wp_mid_point_x = (_wp[0] + _wp_next[0]) / 2
        wp_mid_point_y = (_wp[1] + _wp_next[1]) / 2
        wp_mid_point_z = (_wp[2] + _wp_next[2]) / 2
        if i == 0:
#            wp = Point(_wp[0], _wp[1], _wp[2])        
            wp_next = Point(wp_mid_point_x, wp_mid_point_y, wp_mid_point_z)
        else:
#            wp = Point(wp_mid_point_x, wp_mid_point_y, wp_mid_point_z)
            wp_next = Point(_wp_next[0], _wp_next[1], _wp_next[2])
#        print('EEF TMP POS', wp_next.x, wp_next.y, wp_next.z)

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
    
def compute_obj_pos(obj_pos,
                    wp, 
                    wp_final):

    obj_moved = check_obj_moved(obj_pos,
                                wp, 
                                wp_final)
    new_box_pos = [pos for pos in obj_pos]

#    z_max_value_box = obj_pos[2] + sim_param.obj_side/2  
    
    if obj_moved:
        
#        print('EEF TMP POS', wp[0], wp[1], wp[2])
#        print('EEF TMP NEXT POS', wp_final[0], wp_final[1], wp_final[2])        
        wp = [round(pos, sim_param.round_value) for pos in wp]
        wp_final = [round(pos, sim_param.round_value) for pos in wp_final]
        
        ## compute pos variations
        var_x = abs(wp[0] - wp_final[0])
        var_y = abs(wp[1] - wp_final[1])
        var_z = abs(wp[2] - wp_final[2])
        
        ## bigger variation reflects move orientation
        if var_y >= var_x: ## horizontal move
            if wp[1] > wp_final[1]: ## left
                new_box_pos[1] -= sim_param.obj_displacement
            elif wp[1] < wp_final[1]: ## right
                new_box_pos[1] += sim_param.obj_displacement

        elif var_x >= var_y: ## vertical move        
            if wp[0] < wp_final[0]: ## far
                new_box_pos[0] += sim_param.obj_displacement
            elif wp[0] > wp_final[0]: ## close
                new_box_pos[0] -= sim_param.obj_displacement        
    
    new_box_pos = [round(pos,sim_param.round_value) for pos in new_box_pos]
    return obj_moved, new_box_pos

'''
Given a effect return the related final position
'''
def identify_final_pos(_obj_pos, _effect):
    if _effect == 'far':
        return [_obj_pos[0] + sim_param.obj_displacement, _obj_pos[1], 0]
    elif _effect == 'close':
        return [_obj_pos[0] - sim_param.obj_displacement, _obj_pos[1], 0]
    elif _effect == 'left':
        return [_obj_pos[0], _obj_pos[1] + sim_param.obj_displacement, 0]
    elif _effect == 'right':
        return [_obj_pos[0], _obj_pos[1] - sim_param.obj_displacement, 0] 

'''
Given a final position identify the effect
Virtual coordinates
'''
def identify_effect(initial_obj_pos, final_obj_pos):
    print(initial_obj_pos, final_obj_pos)
    if final_obj_pos[0:2] == \
        [round(initial_obj_pos[0] + sim_param.obj_displacement, sim_param.round_value), 
         initial_obj_pos[1]]:
        return 'far'        
    elif final_obj_pos[0:2] == \
        [round(initial_obj_pos[0] - sim_param.obj_displacement, sim_param.round_value), 
         initial_obj_pos[1]]:
        return 'close'
    elif final_obj_pos[0:2] == \
        [initial_obj_pos[0], 
         round(initial_obj_pos[1] + sim_param.obj_displacement, sim_param.round_value)]:
        return 'left'        
    elif final_obj_pos[0:2] == \
        [initial_obj_pos[0], 
         round(initial_obj_pos[1] - sim_param.obj_displacement, sim_param.round_value)]:
        return 'right'
    else:
        print('ERROR - identify_effect : no effect identified')
        return ''
        
'''
Given a final position identify the effect
Real coordinates
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

   
#'''
#Given a effect return the related final position
#'''
#def identify_effect_final_pos(_obj_pos, _effect):
#
#    if sim_param.inversed_effects:
#        if _effect == 'right':
#            _effect = 'left'
#        elif _effect == 'left':
#            _effect = 'right'
#        elif _effect == 'up':
#            _effect = 'down'
#        elif _effect == 'down':
#            _effect = 'up'            
#    
#    if _effect == 'right':
#        return [_obj_pos[0] + sim_param.obj_displacement, _obj_pos[1], 0]
#    elif _effect == 'left':
#        return [_obj_pos[0] - sim_param.obj_displacement, _obj_pos[1], 0]
#    elif _effect == 'up':
#        return [_obj_pos[0], _obj_pos[1] + sim_param.obj_displacement, 0]
#    elif _effect == 'down':
#        return [_obj_pos[0], _obj_pos[1] - sim_param.obj_displacement, 0]    
   
'''
Test
'''
if __name__ == '__main__':
    
    new_pos = compute_obj_pos([0.65,-0.05,-0.09], [0.65, -0.04, -0.13], [0.62, -0.07, -0.15])
    print(new_pos)
    
#    print(identify_effect([0.65,-0.05,-0.09], new_pos[1]))
    
    
#    print(identify_effect([-0.3,0,0]))
#    print(identify_effect([0,0,0], [0.3,0,0])) ## far
#    print(identify_effect([0,0,0], [-0.3,0,0])) ## close
#    print(identify_effect([0,0,0], [0,0.3,0])) ## left
#    print(identify_effect([0,0,0], [0,-0.3,0])) ## right

#    print(check_obj_moved([0,0,0], [0,0.5,0],[0,0.01,0])) # to the bottom
#    print(check_obj_moved([0,0,0], [0,-0.5,0],[0,-0.01,0])) # up
#    print(check_obj_moved([0,0,0], [0.5,0,0],[0.01,0,0])) # to the left
#    print(check_obj_moved([0,0,0], [-0.5,-0,0],[-0.01,0,0])) # to the right
#    
#    print(check_obj_moved([0,0,0], [-0.2,0,0],[0,-0.2,0])) # to the right
#    print(check_obj_moved([0,0,0], [0.2,0,0],[0,-0.2,0])) # to the left
#    print(check_obj_moved([0,0,0], [0.2,0,0],[0,1,0])) # to the left
#    print(check_obj_moved([0,0,0], [-0.1,-1,0],[0.1,10,0])) # up

#    print(check_obj_moved([0.65, 0.1, -0.11], 
#                          [0.7, 0.1, -0.11],[0.7, 0.15, -0.11]))