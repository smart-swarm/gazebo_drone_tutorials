#!/usr/bin/env python
import math

from player_interface import PlayerInterface



class Example(PlayerInterface):
    def __init__(self): #### initialization 
        super(Example, self).__init__()
        
    def deal_move(self): #### design the move strategy   
        for name in self.get_self_agent_name_list():        
            if 'uav' in name:
                angular_z = 0
                linear_x = 1
                linear_y = 0
                linear_z = 0
                 
                cmd = [linear_x, linear_y, linear_z, angular_z]
                self.set_move_cmd(name, cmd)
            elif 'car'in name:
                cmd=[0,0,0,0]
                self.set_move_cmd(name, cmd)
        self.move_cmd_send()
   
    def process(self):
        self.deal_move()
      

# Coding stop here.




















#######################################################################
             
    def deal_attack(self): #### design the attack strategy
        target1 = self.get_pose('carB1')[0:3]  #### get the position of CAR B1 of enemy team

        self.attack_cmd_send('uavA1', target1) #### send the attack command

        self.attack_cmd_send('uavA2', target1)

        self.attack_cmd_send('carA1', target1)
   