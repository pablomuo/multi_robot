#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
import numpy as np
import math
import random
import json
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from std_msgs.msg import String, Float32MultiArray, Float64MultiArray
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from pathfinding import pathfinding
from reinforcenment import ReinforcementNetwork
from rooms import Check_room
from target import Target
from math import pi
import numpy
import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from environment import Behaviour

class Agent(object):
    ""
    ""

    def __init__(self,agent_name,action_size,state_size,number_episode,rank,id_forzado,type_robot, min_action_time=0.25):
        self.rank                       = rank
        self.num_laseres                = 0
        self.action_done                = 1
        self.action_done_past           = 1
        self.scan_data_past             = 24*[0]
        self.cont_act                   = 0
        self.cont                       = 0
        self.vel_lineal                 = 0
        self.vel_ang                    = 0
        self.type_robot                 = type_robot
        # self.dirPath            = os.path.dirname(os.path.realpath(__file__))
        self.dirPath            = "/home/mcg/catkin_ws/src/multi_robot/save_model/environment"
        # self.dirPath            = self.dirPath.replace('multi_robot/nodes', 'multi_robot/save_model/environment_')
        self.agent_name                 = agent_name
        # Save the data laser to avoid asking ros all the time
        self.__step_cache_1               = -1
        self.step                         = 0
        self.force_update_1               = False
        self.last_heading                 = []
        self.heading_1                    = 0
        self.heading_r2_r1                = 0
        self.theta_1= 0
        self.__status                     = "follower"
        self.reward                       =0
        self.load                         =random.choice([False,False,False])
        self.step                         = 0
        self.__process                  = "driving_to_goal"
        self.number_action              = action_size
        self.max_angular_vel            = 1
        self.__min_action_time          = min_action_time
        self.__angle_actions            = np.array([((self.number_action-4-action)* self.max_angular_vel*0.5)*self.__min_action_time for action in range(action_size-1)])
        self.old_goal                   = (np.nan,np.nan)
        self.__forward_action           = int((action_size-2)/2.)
        self.__backward_action          = int(action_size-1)
        self.min_range                  = 0.18
        # Avoid countinf multiple crashes en the same step
        self.__crashed                  = False
        self.__crash_counter            = 0
        self.score                      = 0
        self.global_step                = 0
        self.cont                       = 0
        self.__avoid_distance           = 1
        self.__action_time              = 0

        self.diff_time                  = 0.45
        self.done                       = False
        self.finish                     = False
        self.__current_leader           = 0  # number of the rank who is the leader
        self.__ID                       = id_forzado
        self.old_ID                     = 0
        self.room                       = Check_room()
        self.robot_position_y           = 0
        self.robot_position_x           = 0



    def call_sub(self,agent_name,action_size,state_size,number_episode,rank,id_forzado,type_robot):
        print("1", self.agent_name, "ID", self.ID)
        self.unpause_proxy                = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        print("2", self.agent_name)
        self.pause_proxy                  = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        print("3", self.agent_name)
        self.pub_cmd_vel                  = rospy.Publisher(self.agent_name+'/cmd_vel', Twist, queue_size=5)
        print("4", self.agent_name)
        # self.environment                  = Behaviour(agent_name,self.ID,rank)
        print("5", self.agent_name)
        self.sub_odom_1                   = rospy.Subscriber(self.agent_name+'/odom', Odometry, self.get_Odometry_1)
        self.get_Odometry_1
        # print(self.robot_position_x)
        # print(self.robot_position_y)
        print("6", self.agent_name, "ID", self.ID)
        self.environment                  = Behaviour(agent_name,self.ID,rank)
        self.robot_position_i           = Pose()
        print("7", self.agent_name)
        # self.robot_position_y           = 0
        # self.robot_position_x           = 0
        self.pub_scan                     = rospy.Publisher(self.agent_name+'/scan_out', Float64MultiArray, queue_size=24)
        print("8", self.agent_name)
        self.__pathfinding                = pathfinding(self.laser_angles/360.*2*np.pi)
        print("9", self.agent_name)
        self.learning                     = ReinforcementNetwork(state_size,action_size,number_episode,self.load,0)
        print("10", self.agent_name)
        self.start_time                   = time.time()
        print("11", self.agent_name)
        self.learning.get_Pa()
        print("12", self.agent_name)
        self.state_initial                = None



    @property
    def ID(self):
        return self.__ID

    @ID.setter
    def ID(self,value):
        self.__ID = value
        self.environment.ID=value
        self.environment.target_position.ID=value

    @property
    def current_leader(self):
        return self.__current_leader

    @current_leader.setter
    def current_leader(self,value):
        self.__current_leader = value
        if value==self.rank:
            self.status="lead"
        else:
            self.status="follower"

    @property
    def get_initial_status(self):
        self.state_initial,_ = np.array(self.state)
        return self.state_initial

    @property
    def status(self):
        return self.__status

    @status.setter
    def status(self,value):
        self.__status = value

    def get_Odometry_1(self, odom):
        '''
        Position and orientation to the robot
        '''
        self.robot_position_i = odom.pose.pose.position
        self.robot_position_x =self.robot_position_i.x
        self.robot_position_y =self.robot_position_i.y
        robot_1_angle = odom.pose.pose.orientation
        angles_robot_list_1 = [robot_1_angle.x, robot_1_angle.y, robot_1_angle.z, robot_1_angle.w]
        _, _, yaw = euler_from_quaternion(angles_robot_list_1)
        self.theta_1 = yaw
        self.environment.goal_angle = math.atan2(self.environment.goal_y - self.robot_position_y, self.environment.goal_x - self.robot_position_x)
        self.heading_1 = self.environment.goal_angle - yaw

        if self.heading_1 > pi:
            self.heading_1 -= 2 * pi
        elif self.heading_1 < -pi:
            self.heading_1 += 2 * pi
        self.last_heading.append(self.heading_1)
        # print("last heading "+str(self.agent_name)+": ", self.heading_1)


    @property
    def scan_data(self):

        if self.type_robot >= 5:
            '''
            Get data of the laser
            '''
            if self.__step_cache_1 == self.step and not self.force_update_1:
                scan_data=self.__scan_data_cache_1
            else:
                data = None
                while data is None:
                    try:
                        data = rospy.wait_for_message('/'+self.agent_name+'/scan', LaserScan, timeout=5)
                        # data = rospy.wait_for_message('/camera_sync_1cam_laser', LaserScan, timeout=5)
                    except:
                        pass
                scan = data
                scan_data = []
                for i in range(len(scan.ranges)):
                    if scan.ranges[i] == float("Inf"):
                        scan_data.append(3.5)
                    elif np.isnan(scan.ranges[i]):
                        scan_data.append(0)
                    else:
                        scan_data.append(scan.ranges[i])
                if np.any(np.isnan(np.array(scan_data))):
                    raise Exception("it's nan sensor")

                self.__step_cache_1     = self.step
                self.__scan_data_cache_1 = scan_data
                self.force_update_1      = False
                scan_data2 = Float64MultiArray()
                scan_data12=np.array(scan_data)
                scan_data2.data = scan_data12
                self.pub_scan.publish(scan_data2)
        # print(scan_data)
        # return scan_data


        elif self.type_robot == 4:
            '''
            Get data of the two cameras
            '''
            if self.__step_cache_1 == self.step and not self.force_update_1:
                scan_data=self.__scan_data_cache_1
            else:
                data = None
                while data is None:
                    try:
                        data = rospy.wait_for_message('/'+self.agent_name+'/camera_sync', LaserScan, timeout=5)
                        # data = rospy.wait_for_message('/camera_sync', LaserScan, timeout=5)
                    except:
                        pass

                scan = data
                scan_data = []

                # for i in range(len(scan.ranges)):
                #     scan_data.append(scan.ranges[i])

                for i in range(len(scan.ranges)):
                    scan_data.append(scan.ranges[i])       #se cogen los 48 valores y se envían, fuera se separarán

                if np.any(np.isnan(np.array(scan_data))):
                    raise Exception("it's nan sensor")

                self.__step_cache_1     = self.step
                self.__scan_data_cache_1 = scan_data
                self.force_update_1      = False
                scan_data2 = Float64MultiArray()
                scan_data12=np.array(scan_data)
                scan_data2.data = scan_data12
                self.pub_scan.publish(scan_data2)



        else:
            '''
            Get data of the camera
            '''
            if self.__step_cache_1 == self.step and not self.force_update_1:
                scan_data=self.__scan_data_cache_1
            else:
                data = None
                while data is None:
                    try:
                        data = rospy.wait_for_message('/'+self.agent_name+'/camera_laser_sync', LaserScan, timeout=5)
                        # data = rospy.wait_for_message('/camera_sync', LaserScan, timeout=5)
                    except:
                        pass

                scan = data
                scan_data = []

                # for i in range(len(scan.ranges)):
                #     scan_data.append(scan.ranges[i])

                for i in range(len(scan.ranges)):
                    scan_data.append(scan.ranges[i])       #se cogen los 48 valores y se envían, fuera se separarán

                if np.any(np.isnan(np.array(scan_data))):
                    raise Exception("it's nan sensor")

                # scan_data = scan_data[0:24]

                #------------------------------------------------------------------------------------------------------------------

                self.num_laseres = scan_data[48]

                val_a_cambiar = int(math.floor(self.num_laseres/2))+1     #indica cual es la primera posicion a modificar, si es camara de 90 grados, 7 laseres, primera posicion scan_data[4]
                if sum(self.scan_data_past) != 0:

                    dist_rec = self.diff_time*abs(self.vel_lineal)     #distancia que recorre --> tiempo x velocidad lineal
                    # print("dist_rec", dist_rec)
                    if self.vel_ang != 0:
                        radio = abs(self.vel_lineal/self.vel_ang)
                        dist_ang = dist_rec/radio                      # desplazamiento angular en radianes
                        ang1 = (math.radians(180) - dist_ang)/2        # el traingulo formado por dos puntos de una circunferencias es siempre isosceles (2 lados son el radio)
                        ang2 = math.radians(90) - ang1                 # queremos el angulo complementario de ang1
                        dist_lineal = self.get_point_position(radio, radio, dist_ang, 0)
                    else:
                        ang2 = 0
                        pass

                    # print("data_old", scan_data[0:24])
                    cont_ant = self.cont
                    # print("contador", self.cont)
                    if self.action_done_past != self.action_done and self.action_done_past == 2:     #cuando proviene de una accion 2
                        self.reset_cont(20-val_a_cambiar-type_robot)
                        scan_data[val_a_cambiar:(val_a_cambiar+cont_ant)] = self.scan_data_past[val_a_cambiar:(val_a_cambiar+cont_ant)]
                        scan_data[23-val_a_cambiar:(23-val_a_cambiar-cont_ant)] = self.scan_data_past[23-val_a_cambiar:(23-val_a_cambiar-cont_ant)]
                        scan_data[10:14] = self.scan_data_past[10:14]
                        # for i in range(self.cont):
                        #     scan_data[val_a_cambiar+i]  = self.scan_data_past[val_a_cambiar+i]
                        #     scan_data[23-val_a_cambiar-i] = self.scan_data_past[23-val_a_cambiar-i]

                    elif self.action_done < 2 :
                        self.reset_cont(20-val_a_cambiar-type_robot)
                        if abs(self.action_done - self.action_done_past) == 1:    #cuando esta en 0 y viene de 1, y viceversa
                            scan_data[23-val_a_cambiar:(23-val_a_cambiar-self.cont)] = self.scan_data_past[23-val_a_cambiar:(23-val_a_cambiar-self.cont)]

                        elif abs(self.action_done - self.action_done_past) > 1:    #cuando viene de la accion 3, 4
                            # self.reset_cont(14-val_a_cambiar)
                            if cont_ant > 14-val_a_cambiar:
                                range_max = 14-val_a_cambiar
                                scan_data[14:val_a_cambiar+cont_ant] = self.scan_data_past[14:val_a_cambiar+cont_ant]
                            else:
                                range_max = self.cont

                            for i in range(self.cont):
                                scan_data[val_a_cambiar+i] = self.scan_data_past[val_a_cambiar+i-1]

                        elif self.action_done == 0 and self.action_done_past == self.action_done:
                            for i in range(self.cont):
                                scan_data[23-val_a_cambiar-i] = self.scan_data_past[23-val_a_cambiar-i+1]

                        elif self.action_done == 1 and self.action_done_past == self.action_done:
                            for i in range(self.cont):
                                # scan_data[val_a_cambiar+i]  = self.get_point_position(self.scan_data_past[val_a_cambiar+i-1], dist_rec, (val_a_cambiar+i-1), -ang2)
                                scan_data[23-val_a_cambiar-i] = self.get_point_position(self.scan_data_past[23-val_a_cambiar-i+1], dist_lineal, (23-val_a_cambiar-i+1), ang2)


                    elif self.action_done == 2:
                        self.reset_cont(10-val_a_cambiar)
                        if self.action_done_past == self.action_done:
                            for i in range(self.cont):
                                scan_data[val_a_cambiar+i]  = self.get_point_position(self.scan_data_past[val_a_cambiar+i-1], dist_rec, (val_a_cambiar+i-1), ang2)
                                scan_data[23-val_a_cambiar-i] = self.get_point_position(self.scan_data_past[23-val_a_cambiar-i+1], dist_rec, (23-val_a_cambiar-i+1), ang2)

                            for j in range(4):
                                if j == 0 or j == 3:
                                    scan_data[10+j] = self.scan_data_past[10+j] + (dist_rec/math.cos(math.radians(7.5+15)))
                                else:
                                    scan_data[10+j] = self.scan_data_past[10+j] + (dist_rec/math.cos(math.radians(7.5)))

                        elif self.action_done_past < 2:  #cuando viene de la accion 0, 1
                            scan_data[23-val_a_cambiar:23-val_a_cambiar-cont_ant] = self.scan_data_past[23-val_a_cambiar:23-val_a_cambiar-cont_ant]

                        elif self.action_done_past > 2:  #cuando viene de la accion 3,4
                            scan_data[val_a_cambiar:val_a_cambiar+cont_ant] = self.scan_data_past[val_a_cambiar:val_a_cambiar+cont_ant]


                    elif self.action_done > 2 :
                        self.reset_cont(20-val_a_cambiar-type_robot)
                        if abs(self.action_done - self.action_done_past) == 1:    #cuando esta en 3 y viene de 4 y viceversa

                            scan_data[val_a_cambiar:(val_a_cambiar+self.cont)] = self.scan_data_past[val_a_cambiar:(val_a_cambiar+self.cont)]

                        elif abs(self.action_done - self.action_done_past) > 1:    #cuando viene de la accion 0, 1
                            # self.reset_cont(14-val_a_cambiar)
                            if cont_ant > 14-val_a_cambiar:
                                range_max = 14-val_a_cambiar
                                scan_data[10-(cont_ant-14-val_a_cambiar):10] = self.scan_data_past[10-(cont_ant-14-val_a_cambiar):10]
                            else:
                                range_max = self.cont
                            for i in range(range_max):
                                    scan_data[23-val_a_cambiar-i] = self.scan_data_past[23-val_a_cambiar-i+1]

                        elif self.action_done == 3 and self.action_done_past == self.action_done:
                            for i in range(self.cont):
                                scan_data[val_a_cambiar+i]  = self.get_point_position(self.scan_data_past[val_a_cambiar+i-1], dist_lineal, (val_a_cambiar+i-1), ang2)
                                    # scan_data[23-val_a_cambiar-i] = self.get_point_position(self.scan_data_past[23-val_a_cambiar-i+1], dist_rec, (23-val_a_cambiar-i+1), -ang2)

                        elif self.action_done == 4 and self.action_done_past == self.action_done:
                            for i in range(self.cont):
                                scan_data[val_a_cambiar+i]  = self.scan_data_past[val_a_cambiar+i-1]
                                    # scan_data[23-val_a_cambiar-i] = self.scan_data_past[23-val_a_cambiar-i+1]

                            # else: #cuando la accion anterior haya sido 0, 1 (la 2 esta valorada arriba del todo)
                            #     pass
                    else:
                        self.cont = 0

                else:
                    self.cont=0
                    # self.action_done_past = self.action_done
                    pass


                # print("camera")
                # print(scan_data[0:24])
                # print("laser")
                # print(scan_data[24:48])

                self.scan_data_past = scan_data[0:24]
                self.action_done_past = self.action_done

                #------------------------------------------------------------------------------------------------------------------

                self.__step_cache_1     = self.step
                self.__scan_data_cache_1 = scan_data
                self.force_update_1      = False
                scan_data2 = Float64MultiArray()
                scan_data12=np.array(scan_data[0:24])
                scan_data2.data = scan_data12
                self.pub_scan.publish(scan_data2)

            # print(scan_data)
            #return scan_data

        return(scan_data)



    def get_point_position(self, lado1, lado2, ang, ang2):
        #formula para calcular el lado de un triangulo no rectangulo, sabiendo dos lados y un angulo
        point_position = np.sqrt(math.pow(lado1,2)+math.pow(lado2,2)-(2*lado1*lado2*math.cos(math.radians(((ang)*15)+7.5)+ang2)))
        return point_position



    def reset_cont(self, num_rep):
        if self.action_done_past == 2 or abs(self.action_done - self.action_done_past) <= 1:   # self.action_done - self.action_done_past) <= 1 contempla cuando (self.action_done == self.action_done_past) y hace acciones hacia el mismo lado
           self.cont+=1
           if self.cont > num_rep:
            self.cont = num_rep
        else:
           self.cont = 1



    def perform_action(self,action):
        '''
        Calculates the time needed to excecute the actions
        and executes them
        '''
        # action = 3
        # self.cont_act+=1
        # lrn= 7
        # if self.cont_act < lrn:
        #     action = 1
        # elif self.cont_act >=lrn and self.cont_act <=lrn+5:
        #     action = 2
        # elif self.cont_act > lrn+5 and self.cont_act <=lrn+10:
        #     action = 3
        # elif self.cont_act >lrn+10:
        #     action = 2

        self.diff_time = time.time()-self.__action_time
        print(self.diff_time)

        #print("Tiempo que tarda en realizar la accion:", self.diff_time)
        self.__action_time = time.time()
        max_angular_vel = 1.5
        if action != self.__backward_action:
            ang_vel = ((self.number_action - 1)/2 - action) * max_angular_vel * 0.5

        vel_cmd = Twist()


        if action == self.__backward_action:
            vel_cmd.linear.x = -0.15
            vel_cmd.angular.z = 0
            self.vel_lineal = -0.15
            self.vel_ang = 0

        elif action == self.__forward_action:
            vel_cmd.linear.x = 0.15
            vel_cmd.angular.z = 0
            self.stand=False
            self.vel_lineal = 0.15
            self.vel_ang = 0

        else:
            vel_cmd.linear.x = 0.12
            vel_cmd.angular.z = ang_vel
            self.vel_lineal = 0.12
            self.vel_ang = ang_vel

            self.vel_cmd=vel_cmd
        self.pub_cmd_vel.publish(vel_cmd)

        self.action_done = action
        print("accion", action)
        # print("angulo", ang_vel)

    def reset(self):
        """
          Reset the robot
        """
        self.__process      = "driving_to_goal"
        self.__free_counter = 0
        self.__coll_count   = 0


    @property
    def position(self):
        return (self.robot_position_x, self.robot_position_y)

    @property
    def process(self):
        return self.__process

    @process.setter
    def process(self,value):
        self.__process = value

    @property
    def angle_action(self):
        # return np.array([((self.number_action-4-action)* self.max_angular_vel*0.5)*self.diff_time for action in range(self.number_action-1)])
        return np.array([((self.number_action-4-action)* self.max_angular_vel*0.5)*self.diff_time for action in range(self.number_action-1)])

    @property
    def laser_angles(self):
        """
          Returns the angles of the laser
        """
        scan_data = self.scan_data[0:24]
        angles = 360./(len(scan_data)-1)*np.arange(len(scan_data))
        angles[angles>180] = angles[angles>180]-360
        # print("hasta aqui", self.agent_name)
        return angles

    def rotate_to_angle(self):
        """
          Rotate to a given angle
        """
        eta=0
        # aa=self.__angle_actions+eta*self.__angle_actions
        aa=self.angle_action+eta*self.angle_action
        diff_angles = self.heading_1-self.__desired_angle
        if diff_angles<0:
            mask   = ((diff_angles-aa)<0)
        elif diff_angles>=0:
            mask   = (diff_angles-aa)>=0
        if abs(diff_angles) <np.deg2rad(10):
            helper = np.argmin(abs(diff_angles-aa[mask]))
            mask2=np.ones(len(aa),dtype=bool)
        else:
            mask2 =( np.arange(len(aa))==1 )| (np.arange(len(aa))==3)
            helper = np.argmin(abs(diff_angles-aa[mask&mask2]))
        action = np.arange(len(aa))[mask&mask2][helper]
        # print("change_angle",action,self.heading-self.__desired_angle)
        return action, (action==2)

    @property
    def free_dist_goal(self):
        """
        Calculates the free distance to the goal, using laser information
        """
        scan_data = np.array(self.scan_data[0:24])
        sortkey = np.argsort(abs(np.rad2deg(self.heading_1)-self.laser_angles))[0:3]

        return np.min(scan_data[sortkey])

    @property
    def status_regions(self):
        scan_data =self.scan_data[0:24]
        regions = {
        'right':  min(scan_data[16:20]),
        'sright':  max(scan_data[15:19]),
        'fright': min(scan_data[18:22]),
        'front':  min(min(scan_data[21:25]), min(scan_data[0:4])),
        'fleft':  min(scan_data[2:6]),
        'left':   min(scan_data[5:9]),
        'sleft':   max(scan_data[5:9]),
        'backl':  scan_data[11],
        'backr':  scan_data[13],
        'back':  scan_data[12],
        'stop': min(min(scan_data[21:25]),min(scan_data[0:3])) }
        return regions

    def change_process(self, finished):
        """
          Change the optimal process to reach the goal
        """
        # print("i am in change_process")

        free_dist = self.free_dist_goal
        goal_dist = self.environment.get_current_Distance(self.robot_position_x,self.robot_position_y)
        if finished:
            self.__process = "follow_path"
        elif self.__process=="follow_path":
            # Only drive to goal if the distance to the goal is okay and not blocked
            if (free_dist>goal_dist):
                self.__desired_angle = self.__find_good_angle()
                self.__process="driving_to_goal"
        elif self.__process=="driving_to_goal":
            if self.status_regions["front"]<=self.__avoid_distance/1.5:
                self.__process="follow_path"
        elif self.__process=="collision":
            if self.__coll_count >= 15:
                self.__process="follow_path"


    def __find_good_angle(self):
            '''
            Look for an obstacle free angle
            '''
            scan         = np.array(self.scan_data[0:24])
            laser_angles = np.array(self.laser_angles)
            mask         = scan>self.__avoid_distance
            indices      = np.arange(len(laser_angles))
            ii_g         = np.argsort(abs((np.deg2rad(laser_angles[mask])-self.heading_1)))[0]
            idx          = indices[mask][ii_g]
            towards_goal = np.deg2rad(laser_angles[idx])

            res = towards_goal
            if idx+1>=len(scan):
                nidx = -1
            else:
                nidx = idx+1

            if scan[idx-1]>scan[nidx]:
                res = res+np.deg2rad(laser_angles[idx]-laser_angles[idx-1])
            else:
                res = res+np.deg2rad(laser_angles[idx]-laser_angles[nidx])
            res=self.environment.fix_angle(res-self.heading_1)

            return res


    def evolve(self):
        """
          Make one step with the robot
        """
        scan = np.array(self.scan_data[0:24])
        # Update the map, probably not necessary every step
        if (self.step % 1 == 0) and (self.step!=0):
            self.__pathfinding.update_map(self.position,self.environment.target_position.position,self.heading_1,scan)
        # Construct a path
        if (self.old_goal != self.environment.target_position.position) or (self.step % 5 == 0) and (self.__process == "follow_path"):
            self.__pathfinding.construct_path(self.position,self.environment.target_position.position)

        # Finish the actual __process
        finished = False
        if self.__process=="collision":
            laser_angles = np.array(self.laser_angles)
            #laser_angles expressed in degrees
            if abs(laser_angles[np.argmin(scan)])>90:
                action = self.__forward_action
            else:
                action = self.__backward_action
            self.__coll_count +=1
        elif self.__process == "follow_path":
            self.__desired_angle = self.__pathfinding.follow_path(self.position,self.environment.target_position.position)
            action,_  = self.rotate_to_angle()
        elif self.__process== "driving_to_goal":
            self.__desired_angle = 0
            action,_  = self.rotate_to_angle()
        elif self.__process == "change_angle":
            action,finished = self.rotate_to_angle()

        # For debugging
        self.__pathfinding.monitor(self.position,self.environment.target_position.position)
        # Change the process
        self.change_process(finished)
        self.old_goal = self.environment.target_position.position
        self.action = action
        return self.action


    @property
    def state(self):
        '''
        Get state of the robot goal_angle heading,  scan_data,current_distance
        '''
        current_distance= self.environment.get_current_Distance(self.robot_position_x,self.robot_position_y)
        heading = self.heading_1


        scan_data = self.scan_data[0:24]
        if self.type_robot <= 3:
            check_laser = self.scan_data[24:48]
        else:
            check_laser = self.scan_data[0:24]

        if ((self.min_range >= min(check_laser) > 0) and (not self.__crashed)) \
            or ((self.min_range >= min(check_laser) > 0) and (self.__crash_counter>5))  :
            print("collision detected by laser, position "+str(self.agent_name)+": ", min(check_laser),self.min_range )
            sys.stdout.flush()

            self.done = True
            self.__crashed = True
            self.__crash_counter = 0
            self.__process = "collision"
            self.__coll_count = 0
        elif ((self.min_range >= min(scan_data) > 0) and (not self.__crashed)) \
            or ((self.min_range >= min(scan_data) > 0) and (self.__crash_counter>5))  :
            print("collision detected by camera, position "+str(self.agent_name)+": ", min(scan_data),self.min_range )
            sys.stdout.flush()

            self.done = True
            self.__crashed = True
            self.__crash_counter = 0
            self.__process = "collision"
            self.__coll_count = 0
        elif (self.min_range >= min(scan_data) > 0) and (self.__crashed):
             self.__crash_counter += 1
        elif (self.min_range < min(scan_data)):
            self.__crashed = False

        wall_dist = min(self.scan_data[0:24])
        obstacle_angle = np.argmin(self.scan_data[0:24])

        goal_heading_initial= self.last_heading[0]

        return scan_data + [heading, current_distance,wall_dist,goal_heading_initial], self.done

    def next_values(self):
        '''
        Call reward function and return next_state, reward,done
        '''
        # done                = self.done
        state, done         = self.state
        # print("next_values ",done)
        state               = np.array(state)
        self.reward         = self.environment.set_reward(state, done, self.action,self.step)
        next_state          = np.array(state)
        next_state[0:24]    = next_state[0:24]/3.5
        # next_state[24]      = (next_state[24]+np.pi)/(2*np.pi)
        next_state[24]      = next_state[24]
        next_state[25]      = next_state[25]/self.environment._goal_distance_initial
        next_state[26]      = next_state[26]/3.5
        next_state[27]      = next_state[27]
        # next_state[27]      = (next_state[27]+np.pi)/(2*np.pi)

        self.next_state     = next_state
        self.done           = done
        return self.next_state

    def get_action_value(self):
        # state, _         = np.array(self.state)
        self.action, self.evolve_rule = self.learning.get_action(np.array(self.state_initial))

    def save_data(self,rank,step,e):
        if not os.path.exists(self.dirPath +"_value_agent_"+str(rank)+'.txt'):
            with open(self.dirPath+"_value_agent_"+str(rank)+'.txt', 'a') as outfile:
                outfile.write("step".rjust(8," ")+ "   "+"episode".rjust(8," ")\
                + "   "+"a".rjust(1," ")+"   "+"   "+"reward".rjust(10," ")\
                +"   "+"score".rjust(10," ")+"   "+"robot_x".rjust(10," ")\
                +"   "+"robot_y".rjust(10," ")+"  "+"goal_x".rjust(10," ")\
                +"   "+"goal_y".rjust(10," ") +"   " +"e_r".rjust(1," ")\
                +"   "+"q_value".rjust(8," ")+"   "+"b_time".rjust(10," ")\
                +"   " +"win".rjust(4," ")+"   " +"fail".rjust(4," ")\
                +"   " +"Pa".rjust(10," ")\
                +"   "+"t_h".rjust(2," ")+"   "+"t_m".rjust(2," ")\
                +"   "+"t_s".rjust(2," ")+"   "+"state".rjust(150," ")\
                +"   "+"next_state".rjust(200," ")+"\n")
        m, s = divmod(int(time.time() - self.start_time), 60)
        h, m = divmod(m, 60)
        with open(self.dirPath +"_value_agent_"+str(rank)+'.txt', 'a') as outfile:
            outfile.write("{:8d}".format(step)+"   "+"{:8d}".format(e)\
            +"   "+str(self.action)+"   "+"   "+"{: .3e}".format(self.reward)\
            +"   "+"{: .3e}".format(self.score)+"   "+"{: .3e}".format(self.robot_position_x)\
            +"   "+"{: .3e}".format(self.robot_position_y)+"   "+"{: .3e}".format(self.environment.goal_x) \
            +"   "+"{: .3e}".format(self.environment.goal_y) +"   " +str(int(self.evolve_rule))\
            +"   "+"{: .3e}".format(np.max(self.learning.q_value))+"   "+"{: .3e}".format(self.environment.best_time)\
            +"   " +str(int(self.environment.get_goalbox))+"   " +str(int(self.done))\
            +"   "+"{: .3e}".format(self.learning.Pa)\
            +"   "+"{:8d}".format(h)+"   "+"{:02d}".format(m) \
            +"   "+"   "+"{:02d}".format(s) +"   "+' '.join(map(lambda x: "{: .6f}".format(x), self.state_initial))\
            +"   "+"   "+' '.join(map(lambda x: "{: .6f}".format(x), self.next_state))+"\n")

    def save_data_cloud(self,rank,step,e,evolve_rule_cloud,q_value_cloud):
        if not os.path.exists(self.dirPath +"_value_agent_"+str(rank)+'.txt'):
            with open(self.dirPath+"_value_agent_"+str(rank)+'.txt', 'a') as outfile:
                outfile.write("step".rjust(8," ")+ "   "+"episode".rjust(8," ")\
                + "   "+"a".rjust(1," ")+"   "+"   "+"reward".rjust(10," ")\
                +"   "+"score".rjust(10," ")+"   "+"robot_x".rjust(10," ")\
                +"   "+"robot_y".rjust(10," ")+"  "+"goal_x".rjust(10," ")\
                +"   "+"goal_y".rjust(10," ") +"   " +"e_r".rjust(1," ")\
                +"   "+"q_value".rjust(8," ")+"   "+"b_time".rjust(10," ")\
                +"   " +"win".rjust(4," ")+"   " +"fail".rjust(4," ")\
                +"   " +"Pa".rjust(10," ")\
                +"   "+"t_h".rjust(2," ")+"   "+"t_m".rjust(2," ")\
                +"   "+"t_s".rjust(2," ")+"   "+"state".rjust(150," ")\
                +"   "+"next_state".rjust(200," ")+"\n")
        m, s = divmod(int(time.time() - self.start_time), 60)
        h, m = divmod(m, 60)
        with open(self.dirPath +"_value_agent_"+str(rank)+'.txt', 'a') as outfile:
            outfile.write("{:8d}".format(step)+"   "+"{:8d}".format(e)\
            +"   "+str(self.action)+"   "+"   "+"{: .3e}".format(self.reward)\
            +"   "+"{: .3e}".format(self.score)+"   "+"{: .3e}".format(self.robot_position_x)\
            +"   "+"{: .3e}".format(self.robot_position_y)+"   "+"{: .3e}".format(self.environment.goal_x) \
            +"   "+"{: .3e}".format(self.environment.goal_y) +"   " +str(int(evolve_rule_cloud))\
            +"   "+"{: .3e}".format(np.max(q_value_cloud))+"   "+"{: .3e}".format(self.environment.best_time)\
            +"   " +str(int(self.environment.get_goalbox))+"   " +str(int(self.done))\
            +"   "+"{: .3e}".format(self.learning.Pa)\
            +"   "+"{:8d}".format(h)+"   "+"{:02d}".format(m) \
            +"   "+"   "+"{:02d}".format(s) +"   "+' '.join(map(lambda x: "{: .6f}".format(x), self.state_initial))\
            +"   "+"   "+' '.join(map(lambda x: "{: .6f}".format(x), self.next_state))+"\n")

        if  self.environment.get_goalbox == True or self.done==True:
            m, s = divmod(int(time.time() - self.start_time), 60)
            h, m = divmod(m, 60)
            if not os.path.exists(self.dirPath +"_goal_value_agent"+str(rank)+'.txt'):
                with open(self.dirPath+"_goal_value_agent"+str(rank)+'.txt', 'a') as outfile:
                    outfile.write("step".rjust(8," ")+ "   "+"episode".rjust(8," ")\
                    + "   "+"a".rjust(1," ")+"   "+"   "+"reward".rjust(10," ")\
                    +"   "+"score".rjust(10," ")+"   "+"robot_x".rjust(10," ")\
                    +"   "+"robot_y".rjust(10," ")+"  "+"goal_x".rjust(10," ")\
                    +"   "+"goal_y".rjust(10," ") +"   " +"e_r".rjust(1," ")\
                    +"   "+"q_value".rjust(8," ")+"   "+"b_time".rjust(10," ")\
                    +"   " +"win".rjust(4," ")+"   " +"fail".rjust(4," ")\
                    +"   " +"Pa".rjust(10," ")\
                    +"   "+"t_h".rjust(2," ")+"   "+"t_m".rjust(2," ")\
                    +"   "+"t_s".rjust(2," ")+"   "+"state".rjust(150," ")\
                    +"   "+"next_state".rjust(200," ")+"\n")
            with open(self.dirPath +"_goal_value_agent"+str(rank)+'.txt', 'a') as outfile:
                outfile.write("{:8d}".format(step)+"   "+"{:8d}".format(e)\
                +"   "+str(self.action)+"   "+"   "+"{: .3e}".format(self.reward)\
                +"   "+"{: .3e}".format(self.score)+"   "+"{: .3e}".format(self.robot_position_x)\
                +"   "+"{: .3e}".format(self.robot_position_y)+"   "+"{: .3e}".format(self.environment.goal_x) \
                +"   "+"{: .3e}".format(self.environment.goal_y) +"   " +str(int(evolve_rule_cloud))\
                +"   "+"{: .3e}".format(np.max(q_value_cloud))+"   "+"{: .3e}".format(self.environment.best_time)\
                +"   " +str(int(self.environment.get_goalbox))+"   " +str(int(self.done))\
                +"   "+"{: .3e}".format(self.learning.Pa)\
                +"   "+"{:8d}".format(h)+"   "+"{:02d}".format(m) \
                +"   "+"   "+"{:02d}".format(s) +"   "+' '.join(map(lambda x: "{: .6f}".format(x), self.state_initial))\
                +"   "+"   "+' '.join(map(lambda x: "{: .6f}".format(x), self.next_state))+"\n")



    def append_memory(self):
        self.learning.append_D(self.state_initial, self.action, self.reward, self.next_state, self.done)
        self.learning.append_EPS(self.state_initial, self.action, self.reward, self.next_state, self.done)
        self.state_initial  = self.next_state
        self.score += self.reward

    def work_out_best_state(self):
        if self.environment.get_goalbox == True:
            rospy.loginfo("goal achieved by  %s",self.agent_name)
            sys.stdout.flush()

            if self.environment.best_time > 0.85:
                self.learning.best_state()
            else:
                self.learning.winning_state()
            self.keep_or_reset()
        else:
            pass

    def keep_or_reset(self):
        if self.learning.Pa < self.learning.normal_process:
            # Calculate the distance to the target from the agent's last position
            self.environment.get_goalbox = False
            self.environment.goal_x, self.environment.goal_y = self.environment.target_position.getPosition(True, delete=True)
            self.last_heading = []
            self.environment.get_Distance_goal(self.robot_position_x,self.robot_position_y)

        else:
            # Returns to the agent's origin position and calculates the distance to the target
            self.environment.get_goalbox = False
            self.reset()
            self.environment.reset_gazebo()
            # self.environment.reset(self.robot_position_x, self.robot_position_y)

            self.last_heading=[]

            self.environment.goal_x, self.environment.goal_y = self.environment.target_position.getPosition(True, delete=True)
            self.environment.get_Distance_goal(self.robot_position_x,self.robot_position_y)


    def update_variables(self):
        self.score += self.reward
        self.state = self.next_state
        self.global_step += 1

    def Done(self,done,finish):
        self.learning.update_target_network()
        if finish:
            finish = False
            out= "break"
        if self.evolve_rule:
            self.process="collision"
            self.cont+=1
            self.learning.increase_fact()
            self.last_heading=[]
            self.environment.reset(self.robot_position_x,self.robot_position_y)
            if self.cont > 20:
                self.cont=0
                out= "break"
        else:
            out= "break"
        return out


    def time_to_update(self,q_model_theta,target_model_theta):
        # self.global_step += 1
        # if self.global_step % self.learning.target_update == 0:
        self.learning.update_target_network(q_model_theta,target_model_theta)

    def time_out(self,step):
        if step >= 700:
            rospy.loginfo("Time out!! of %s", self.agent_name )
            # self.done = True
            return True
        else:
            return False

    def check_room(self):
        self.old_ID = self.ID
        self.ID=self.room.check_room(self.robot_position_x,self.robot_position_y)
