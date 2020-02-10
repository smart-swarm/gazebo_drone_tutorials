#!/usr/bin/env python
import math

from player_interface import PlayerInterface



class Example(PlayerInterface):
    def __init__(self): #### initialization 
        super(Example, self).__init__()
        
    def deal_move(self): #### design the move strategy   
        angular_z = 0     ####or more simple:cmd = [1,0,0,0]
        linear_x = 1
        linear_y = 0
        linear_z = 0
        cmd = [linear_x, linear_y, linear_z, angular_z]
        self.set_move_cmd('uavA1', cmd)
        cmd=[0,0,0,0]     
        self.set_move_cmd('uavA2', cmd)
        self.set_move_cmd('uavA3', cmd)
        self.set_move_cmd('carA1', cmd)
        self.set_move_cmd('carA2', cmd)
        self.set_move_cmd('carA3', cmd)
        self.move_cmd_send()
   
    def process(self):
        self.deal_move()
      

# Coding stop here.









   
