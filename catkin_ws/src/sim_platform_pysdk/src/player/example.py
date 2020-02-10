#!/usr/bin/env python
import math

from player_interface import PlayerInterface
import plan_points


class Example(PlayerInterface):
    def __init__(self): #### initialization 
        super(Example, self).__init__()
        
        self.k_p_xyz = 0.1
        self.k_p_yaw = 0.5
        self.v_max = 0.5
        self.plan_points_dict = plan_points.Target

    def get_next_target(self, name):  #### get the next position point
		if name in self.plan_points_dict.keys():
			if self.plan_points_dict[name]:
				return self.plan_points_dict[name][0]
			else:
				return None
		else:
			raise Exception("Error name", name)

    def refresh_target(self, name): #### delete the arrived point
        if name not in self.get_self_alive_name_list():
            return

        state = self.get_pose(name)
        if not self.plan_points_dict[name]:
			return
        target = self.plan_points_dict[name][0]
        err_x = (target[0] - state[0])
        err_y = (target[1] - state[1])
        err_z = (target[2] - state[2])
	
        judge = ((math.fabs(err_x) < 0.4) and (math.fabs(err_y) < 0.6) and (math.fabs(err_z)<1.0))  #### set the control error threshold
        if judge:
            del self.plan_points_dict[name][0]

    def refresh_all_targets(self):  #### update the next position point 
        for name in self.plan_points_dict.keys():
			self.refresh_target(name)

    def is_arrived(self, name):  ####  check the arrived status of one agent
        return False if self.plan_points_dict[name] else True

    def is_all_arrived(self): #### check all the arrived status of all agents
		for name in self.plan_points_dict.keys():
			if not self.is_arrived(name):
				return False
		return True

    def max_limit(self, cmd, v_max): #### set the max velocity
		v_mod = math.sqrt(cmd[0]**2 + cmd[1]**2 + cmd[2]**2)
		if v_mod > v_max:
			for i in range(3):
				cmd[i] = cmd[i]/v_mod*v_max
		return cmd      

    def get_self_alive_name_list(self): #### get the alive agent name of my team
        list_tmp = []
        for name in self.get_self_agent_name_list():
            if self.is_alive(name):
                list_tmp.append(name)
        return list_tmp

    def get_enemy_alive_name_list(self): #### get the alive agent name of enemy team
        list_tmp = []
        for name in self.get_enemy_agent_name_list():
            if self.is_alive(name):
                list_tmp.append(name)
        return list_tmp
#######################################################################
# Coding Here !!!
    # Main process    

    def deal_move(self): #### design the move strategy
        for name in self.get_self_alive_name_list():  #### get the 
            target = self.get_next_target(name) #### get the target position
            if not target: 
                continue
            state = self.get_pose(name) #### get the current position and state
            if 'uav' in name:  ##### Design PID Controller 
                linear_x = self.k_p_xyz*(target[0] - state[0])
                linear_y = self.k_p_xyz*(target[1] - state[1])
                linear_z = self.k_p_xyz*(target[2] - state[2])
                angular_z = self.k_p_yaw*(target[3] - state[5])

                cmd = [linear_x, linear_y, linear_z, angular_z]
                cmd = self.max_limit(cmd, self.v_max)
                self.set_move_cmd(name, cmd)
            if 'car' in name:
                err_x = target[0] - state[0]
                err_y = target[1] - state[1]
                err_dist = math.sqrt(err_x**2 + err_y**2)
                heading_angle = math.atan2(target[1] - state[1], target[0] - state[0])
                err_angle = heading_angle - state[5]
                if math.fabs(err_angle) < 0.1:
                    linear_x = self.k_p_xyz * err_dist
                    linear_y = 0
                    linear_z = 0
                    angular_z = 0
                else:
                    linear_x = 0
                    linear_y = 0
                    linear_z = 0
                    angular_z = self.k_p_yaw * err_angle
                cmd = [linear_x, linear_y, linear_z, angular_z]
                cmd = self.max_limit(cmd, self.v_max)
                self.set_move_cmd(name, cmd)

                
    def deal_attack(self): #### design the attack strategy
        target1 = self.get_pose('carB1')[0:3]  #### get the position of CAR B1 of enemy team

        self.attack_cmd_send('uavA1', target1) #### send the attack command

        self.attack_cmd_send('uavA2', target1)

        self.attack_cmd_send('carA1', target1)
   
    def process(self):
        if self.is_all_arrived():
            self.deal_attack()
        else:
            self.deal_move()
        self.move_cmd_send()
        self.refresh_all_targets()

# Coding stop here.
#######################################################################
