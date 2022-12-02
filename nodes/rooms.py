import numpy as np


class Check_room(object):
    """docstring for ."""


   # MAPA CREADO CON 2 ROOMS

    def __init__(self):
        # HABITACION GRANDE
        # self.room_0_x =[-9.72,10.311]
        # self.room_0_y =[-0.237,9.613]
        # self.room_1_x =[-9.72,10.311]
        # self.room_1_y =[-10.087,-0.237]
        # DOS ROOMS SMALL
        self.room_0_x =[-2.839,3.013]
        self.room_0_y =[-0.13,5.87]
        self.room_1_x =[-2.839,3.013]
        self.room_1_y =[-5.73,-0.14]

    def check_room(self,x_robot,y_robot):
        # para dos habitaciones
        if (self.room_0_x[1]>= x_robot >= self.room_0_x[0]) and\
           (self.room_0_y[1]>= y_robot >= self.room_0_y[0]):
           return 0
        elif (self.room_1_x[1]>= x_robot >= self.room_1_x[0]) and\
              (self.room_1_y[1]>= y_robot >= self.room_1_y[0]):
           return 1
        else:
            raise Exception("There is no room, check your data x: "+str(x_robot)+" y: "+str(y_robot))

        #para una habitacion
        # return 0
