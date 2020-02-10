#!/usr/bin/env python
import math

from player_interface import PlayerInterface



class Example(PlayerInterface):
    def __init__(self): #### initialization 
        super(Example, self).__init__()
             
    def deal_attack(self): #### design the attack strategy
        target1 = self.get_pose('carB1')[0:3]  #### get the position of CAR B1 of enemy team

        self.attack_cmd_send('uavA1', target1) #### send the attack command

        ####self.attack_cmd_send('uavA2', target1)

        ####self.attack_cmd_send('carA1', target1)    
    
   
    def process(self):
        self.deal_attack()
      






















#