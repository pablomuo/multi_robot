#!/usr/bin/env python
import numpy as np
import math
from math import pi
import os
import sys
import random
# sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

class Collaboration(object):
    def __init__(self,agents):
        self.agents = agents
        self.number_agents = len(agents)
        self.distances=np.zeros([self.number_agents,self.number_agents])
        self.headings=np.zeros([self.number_agents,self.number_agents])
        # self.dirPath            = os.path.dirname(os.path.realpath(__file__))
        # self.dirPath            = self.dirPath.replace('collaboration/src/nodes', 'collaboration/src/launch/multi_agent.launch')
        self.path= "/home/pablo/catkin_ws/src/multi_robot/launch/multi_agent.launch"
        self.path1= "/home/pablo/catkin_ws/src/multi_robot/worlds/goal_box/model"
        self.create_file_target()
        self.create_file_robot()

    @property
    def get_heading_r2_r1(self):
        '''
        Calculate the orientation among all agents
        '''
        for i in range(self.number_agents):
            for g in range(self.number_agents):
                if i==g :
                    self.headings[i,g]=np.nan
                else:
                    y1=self.agents[i].robot_1_position_y
                    x1=self.agents[i].robot_1_position_x

                    y2=self.agents[g].robot_1_position_y
                    x2=self.agents[g].robot_1_position_x

                    r2_r1_angle = math.atan2(y1- y2, x1 - x2)

                    heading_r2_r1 = r2_r1_angle - self.agents[g].theta_1
                    if heading_r2_r1 > pi:
                        heading_r2_r1 -= 2 * pi
                    elif heading_r2_r1 < -pi:
                        heading_r2_r1 += 2 * pi
                    self.headings[i,g]=heading_r2_r1

        return self.headings


    @property
    def get_dt_r2_r1(self):
        '''
        Calculate the distances among all agents
        '''

        for i in range(self.number_agents):
            for g in range(self.number_agents):
                if i==g :
                    self.distances[i,g]=np.nan
                else:
                    y1=self.agents[i].robot_1_position_y
                    x1=self.agents[i].robot_1_position_x
                    y2=self.agents[g].robot_1_position_y
                    x2=self.agents[g].robot_1_position_x
                    distance_r2_r1 = math.hypot(x1 - x2, y1 - y2)
                    self.distances[i,g] = distance_r2_r1
        return self.distances

    @property
    def detect_agent(self):
        for i, robot in enumerate(self.agents):
            calculate_only=self.change_leader
            if i==calculate_only:
                # print(i, calculate_only)
                mask=(self.headings[i]<np.deg2rad(180)) & (self.headings[i]!=np.nan)
                mask1=(self.distances[i]<=3.5) & (self.distances[i]!=np.nan)
                headings=np.where(mask)[0]
                distances=np.where(mask1)[0]
                msg=False
                send_agent= None
                get_agent= None
                for j, k in enumerate(distances):
                    if k==headings[j]:
                        send_agent=i
                        get_agent= k
                        msg=True
                    else:
                        msg = False
                        pass
                    print("agent_"+str(send_agent)+ " sent network to agent_"+str(get_agent))
                return msg

    @property
    def change_leader(self):
        reward=np.array([robot.reward for robot in self.agents])
        position=np.argmax(reward)
        for robot in self.agents:
            robot.status = "follower"
        self.agents[position].status="lead"

        return position

    def create_file_robot(self):
        out_str= "<launch>\n"+\
                 '<param name="robot_description"\n'+\
                 'command="$(find xacro)/xacro --inorder $(find turtlebot3_description)/urdf/turtlebot3_burger.urdf.xacro" />"\n'
        x_initial= 1
        Y_initial= 3.13
        ra=random.choice([-1,1])
        for i, agent in enumerate(self.agents):
            out_str += '<group ns="'+agent.agent_name+'">\n'+\
            '<param name="tf_prefix" value="'+agent.agent_name+'_tf" />\n'+\
            '<include file="$(find multi_robot)/launch/one_agent.launch" >\n' + \
            '<arg name="init_pose" value="-x '+str(ra*x_initial+i*ra*0.5)+' -y '+str(ra*x_initial+i*ra*0.5)+' -z 0 -Y '+str(Y_initial-i*0.5)+'" />\n'+\
            '<arg name="agent_name"  value="'+agent.agent_name+'" />\n'+\
            '</include>\n'+\
            '</group>\n\n'
        out_str+='</launch>'
        with open(self.path,"w") as out :
            out.write(out_str)

    def create_file_target(self):
        for i, agent in enumerate(self.agents):
            os.system("pwd")
            os.system("mkdir /home/pablo/catkin_ws/src/multi_robot/worlds/goal_box_"+agent.agent_name)
            print("inside")
            # os.system("/home/pablo/catkin_ws/src/multi_robot/worlds/goal_box_"+agent.agent_name+ "&")
            out_str= "<?xml version='1.0'?>\n"+\
                     "<sdf version='1.6'>\n"+\
                     "  <model name='goal_box_"+agent.agent_name+"'>\n"+\
                     "    <pose frame=''>0 0 0 0 -0 -1.5708</pose>\n"+\
                     "    <link name='goal_box_"+agent.agent_name+"'>\n"+\
                     "      <visual name='goal_box_"+agent.agent_name+"'>\n"+\
                     "        <pose frame=''>0 0 0.0005 0 -0 0</pose>\n"+\
                     "        <geometry>\n"+\
                     "          <box>\n"+\
                     "            <size>0.5 0.5 0.001</size>\n"+\
                     "          </box>\n"+\
                     "        </geometry>\n"+\
                     "        <material>\n"+\
                     "          <script>\n"+\
                     "            <uri>file://media/materials/scripts/gazebo.material</uri>\n"+\
                     "            <name>Gazebo/Grey</name>\n"+\
                     "          </script>\n"+\
                     "          <ambient>1 0 0 1</ambient>\n"+\
                     "        </material>\n"+\
                     "      </visual>\n"+\
                     "      <pose frame=''>0 0 0 0 -0 0</pose>\n"+\
                     "    </link>\n"+\
                     "    <static>1</static>\n"+\
                     "  </model>\n"
            out_str+='</sdf>'
            with open("/home/pablo/catkin_ws/src/multi_robot/worlds/goal_box_"+str(agent.agent_name)+"/model_"+str(agent.agent_name)+".sdf","w") as out :
                out.write(out_str)

            out_str_1= "<?xml version='1.0'?>\n"+\
                     "<model>\n"+\
                     "  <name>goal_box_"+agent.agent_name+"</name>\n"+\
                     "  <version>1.0</version>\n"+\
                     "  <sdf version='1.6'>model_"+agent.agent_name+".sdf</sdf>\n"+\
                     "  <author>\n"+\
                     "      <name></name>\n"+\
                     "      <email></email>\n"+\
                     "  <author>\n"+\
                     "  <description></description>\n"
            out_str_1+='</model>'
            with open("/home/pablo/catkin_ws/src/multi_robot/worlds/goal_box_"+str(agent.agent_name)+"/model_"+str(agent.agent_name)+".config","w") as out_1 :
                out_1.write(out_str_1)
