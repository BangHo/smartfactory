'''
Created on 2023. 11. 24.

@author: WYLee
'''

############################ Load libraries ############################

## grpc libraries (Server)
from __future__ import print_function

from concurrent import futures
import logging

import grpc
import data_pb2
import data_pb2_grpc

## beckhoff libraries (IO)
from io_utils import io_client as client

## wlkata libraries (Robot)
from wlkata_mirobot import WlkataMirobot
from wlkata_mirobot import WlkataMirobotTool

from time import sleep
import time
import json
import threading
import numpy as np
import math
import keyboard
import threading
import matplotlib.pyplot as plt
import asyncio

## Objects
obj = client.InputOutputTask('localhost') # beckhoff object
wlkata_robot1 = WlkataMirobot(portname='/dev/ttyUSB0') # wlkata object
wlkata_robot2 = WlkataMirobot(portname='/dev/ttyUSB1') # wlkata object
wlkata_robot3 = WlkataMirobot(portname='/dev/ttyUSB2') # wlkata object

## Class (grpc server)
class Data(data_pb2_grpc.DataServicer):
    def get_server_status(self, request, context):
        return data_pb2.StrDataReply(strVal=server_status)

    def get_status(self, request, context):
        global status
        return data_pb2.StrDataReply(strVal=status)

    def get_joint_angle(self, request, context):
        global jnt_ang
        response = data_pb2.RepeatedFloatDataReply()
        response.floatVal.extend(jnt_ang)
        return response

    def get_task_position(self, request, context):
        global task_pos
        response = data_pb2.RepeatedFloatDataReply()
        response.floatVal.extend(task_pos)
        return response
    
    def set_robot_task(self, request, context):
        global robot_task_number, input_mode
        input_mode = 0
        robot_task_number = request.intVal
        return data_pb2.StrDataReply(strVal="Task " + str(robot_task_number) + " Sent Done")

    def get_sensor_distance(self, request, context):
        global analog_sensors
        response = data_pb2.RepeatedFloatDataReply()
        response.floatVal.extend(analog_sensors)
        return response

    def homing(self, request, context):
        global ctr_mode
        ctr_mode = 0
        return data_pb2.StrDataReply(strVal='communication done')

    def move_zero(self, request, context):
        global ctr_mode
        ctr_mode = 1
        return data_pb2.StrDataReply(strVal='communication done')

    def move_joint(self, request, context):
        global ctr_mode
        ctr_mode = 2
        return data_pb2.StrDataReply(strVal='communication done')
    
    def get_sensor_dist(self, request, context):
        global is_comm_on
        is_comm_on = True
        return data_pb2.IntDataReply(intVal=result_sensor)
    
    def get_weight_value(self, request, context):
        global weight_mode
        weight_mode = 0
        return data_pb2.FloatDataReply(floatVal=val)

    def terminate_program(self, request, context):
        global is_ctr_on, is_comm_on
        is_ctr_on = False
        is_comm_on = False
        return data_pb2.StrDataReply(strVal='communication done')

    def terminate_server(self, request, context):
        t_serve = threading.Thread(target=self.server_stop)
        t_serve.start()
        return data_pb2.Empty()

    def server_stop(self):
        return server.stop(grace=None)

############################ Define server and device ############################
## Server: SCADA 시스템(Client)의 접속을 허용하기 위한 서버 생성
## Device: Robot, IoT, IO 디바이스들을 운용하기 위한 제어 수행
# GLOBAL_INDICATOR = {'position': True, 'dio': True, 'program': True, 'terminated': False}

# open grpc server
def server():
    global server
    global server_status
    logging.basicConfig()

    port = "50052"
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    data_pb2_grpc.add_DataServicer_to_server(Data(), server)
    server.add_insecure_port("[::]:" + port)
    server.start()
    print("Server started, listening on " + port)
    server_status = "online"
    server.wait_for_termination(timeout=None)

    print("[grpc server] Escape the server\n")


# get digital and analog sensor values
def beckhoff_comm():
    global is_comm_on
    global analog_sensor
    global result_sensor

    analog_sensor = 0
    result_sensor = 0

    is_comm_on = True

    print("Beckhoff ONLINE")

    while is_comm_on:
        analog_sensor = obj.getAnalogInput(0, 0).intVal
        result_sensor = int(str(analog_sensor).strip("[]"))
        sleep(1)

    print("[beckhoff_comm] Escape the program\n")

# control robot - 로드셀에서 신호를 주면 컨베이어 벨트에 박스를 올려주는 로봇팔
def robot_ctr1():
    global is_ctr_on
    global ctr_mode
    global status, jnt_ang, task_pos
    global weight
    global input_mode
    global robot_task_number
    global is_sequence      # sequence 제어를 위한 변수 선언

    is_sequence = False # sequence 변수 초기화
    weight = 0
    is_ctr_on = True
    ctr_mode = -1
    robot_task_number = -1
    
    while is_ctr_on:
        sleep(0.1)
        ctr_mode = robot_task_number
        if wlkata_robot1.get_status().state == 'Idle' and input_mode == 0:
            if ctr_mode == 0:
                print("homing")
                wlkata_robot1.home()
                ctr_mode = -1

            elif ctr_mode == 1:
                print("move zero")
                wlkata_robot1.go_to_zero()
                ctr_mode = -1

            elif ctr_mode == 2 or ctr_mode == 13:
                if weight >= 3 and is_sequence == False:
                    print(weight)
                    wlkata_robot1.linear_interpolation(164.8, -101.8, 85.4, 0, 0, 0, 3000)      # 동작 1
                    time.sleep(0.1)
                    wlkata_robot1.linear_interpolation(164.8, -101.8, 48.4, 0, 0, 0, 3000)
                    time.sleep(0.1)
                    wlkata_robot1.pump_suction()                                                   #석션_on
                    time.sleep(0.1)
                    wlkata_robot1.linear_interpolation(164.8, -101.8, 85.4, 0, 0, 0, 3000)
                    time.sleep(0.1)
                    wlkata_robot1.linear_interpolation(150, 0, 150, 0, 0, 0, 3000)
                    time.sleep(0.1)
                    wlkata_robot1.linear_interpolation(-15, 249.8, 115.7, 0, 0, 0, 3000)
                    time.sleep(0.1)
                    wlkata_robot1.linear_interpolation(-15, 249.8, 90.4, 0, 0, 0, 3000)
                    time.sleep(0.1)
                    wlkata_robot1.pump_off()                                                        #석션_off
                    time.sleep(0.1)
                    wlkata_robot1.linear_interpolation(-15, 249.8, 115.7, 0, 0, 0, 3000)
                    time.sleep(0.1)
                    wlkata_robot1.linear_interpolation(150, 0, 125, 0, 0, 0, 3000)
                    time.sleep(0.1)
                    wlkata_robot1.linear_interpolation(164.8, -101.8, 85.4, 0, 0, 0, 3000)
                    time.sleep(7)
                    is_sequence = True
                    
                else:
                    pass

                # ctr_mode = -1

            elif ctr_mode == 119:
                ctr_mode = -1
                is_sequence = False
                print("Emergency Stop!")
            else:
                ctr_mode = -1

            status = wlkata_robot1.get_status().state
            jnt_ang = [wlkata_robot1.angle.joint1, wlkata_robot1.angle.joint2, wlkata_robot1.angle.joint3,
                       wlkata_robot1.angle.joint4, wlkata_robot1.angle.joint5, wlkata_robot1.angle.joint6]
            task_pos = [wlkata_robot1.cartesian.x, wlkata_robot1.cartesian.y, wlkata_robot1.cartesian.z,
                        wlkata_robot1.cartesian.roll, wlkata_robot1.cartesian.pitch, wlkata_robot1.cartesian.yaw]

    print("[robot_ctr] Escape the program\n")

# 컨베이어 벨트에서 오는 박스를 창고에 팔레타이징 하는 로봇팔
def robot_ctr2():
    global is_ctr_on
    global ctr_mode
    global status, jnt_ang, task_pos
    global is_sequence
    global input_mode
    global robot_task_number

    is_sequence = False
    is_ctr_on = True
    ctr_mode = -1
    robot_task_number = -1
    i = 0
    while is_ctr_on:
        sleep(0.1)
        ctr_mode = robot_task_number
        if wlkata_robot2.get_status().state == 'Idle' and input_mode == 0:
            if ctr_mode == 0:
                print("homing")
                wlkata_robot2.home()
                ctr_mode = -1

            elif ctr_mode == 1:
                print("move zero")
                wlkata_robot2.go_to_zero()
                ctr_mode = -1

            elif ctr_mode == 2 or ctr_mode == 14:
                if is_sequence == True:
                    wlkata_robot2.linear_interpolation(55, -242.8, 106.7, 0, 0, 0, 3000)      # 동작 1
                    time.sleep(0.1)
                    wlkata_robot2.linear_interpolation(55, -242.8, 56, 0, 0, 0, 3000)
                    time.sleep(0.1)
                    wlkata_robot2.pump_suction()                                                   #석션_on
                    time.sleep(0.1)
                    wlkata_robot2.linear_interpolation(55, -242.8, 106.7, 0, 0, 0, 3000)
                    time.sleep(0.1)
                    if i == 0:
                        print("첫번째")
                        wlkata_robot2.linear_interpolation(150, 0, 60, 0, 0, 0, 3000)
                        time.sleep(0.1)
                        wlkata_robot2.linear_interpolation(150, 0, 18.7, 0, 0, 0, 3000)
                        time.sleep(0.1)
                        wlkata_robot2.pump_off()                                                        #석션_off
                        time.sleep(0.1)
                        wlkata_robot2.linear_interpolation(150, 0, 60, 0, 0, 0, 3000)
                        time.sleep(0.1)
                        wlkata_robot2.linear_interpolation(52, -223.9, 106.7, 0, 0, 0, 3000)
                        time.sleep(0.1)
                        i += 1
                        is_sequence = False

                    elif i == 1:
                        print("두번째")
                        wlkata_robot2.linear_interpolation(168.4, 0, 60, 0, 0, 0, 3000)
                        time.sleep(0.1)
                        wlkata_robot2.linear_interpolation(168.4, 0, 18.7, 0, 0, 0, 3000)
                        time.sleep(0.1)
                        wlkata_robot2.pump_off()                                                        #석션_off
                        time.sleep(0.1)
                        wlkata_robot2.linear_interpolation(168.4, 0, 60, 0, 0, 0, 3000)
                        time.sleep(0.1)
                        wlkata_robot2.linear_interpolation(52, -223.9, 106.7, 0, 0, 0, 3000)
                        time.sleep(0.1)
                        i += 1
                        is_sequence = False

                    elif i == 2:
                        print("세번째")
                        wlkata_robot2.linear_interpolation(150, 24.5, 60, 0, 0, 0, 3000)
                        time.sleep(0.1)
                        wlkata_robot2.linear_interpolation(150, 24.5, 18.7, 0, 0, 0, 3000)
                        time.sleep(0.1)
                        wlkata_robot2.pump_off()                                                        #석션_off
                        time.sleep(0.1)
                        wlkata_robot2.linear_interpolation(150, 24.5, 60, 0, 0, 0, 3000)
                        time.sleep(0.1)
                        wlkata_robot2.linear_interpolation(52, -223.9, 106.7, 0, 0, 0, 3000)
                        time.sleep(0.1)
                        i += 1
                        is_sequence = False
                        
                    elif i==3:
                        print("네번째")
                        wlkata_robot2.linear_interpolation(170, 24.5, 60, 0, 0, 0, 3000)
                        time.sleep(0.1)
                        wlkata_robot2.linear_interpolation(170, 24.5, 18.7, 0, 0, 0, 3000)
                        time.sleep(0.1)
                        wlkata_robot2.pump_off()                                                        #석션_off
                        time.sleep(0.1)
                        wlkata_robot2.linear_interpolation(170, 24.5, 60, 0, 0, 0, 3000)
                        time.sleep(0.1)
                        wlkata_robot2.linear_interpolation(52, -223.9, 106.7, 0, 0, 0, 3000)
                        time.sleep(0.1)
                        i = 0
                        is_sequence = False        
                    else:
                        print("Error")              

                else:
                    # print("wait_sequence_signal")
                    pass
            

            elif ctr_mode == 119:
                ctr_mode = -1
                is_sequence = False
                print("Emergency Stop!")
            else:
                ctr_mode = -1

            status = wlkata_robot2.get_status().state
            jnt_ang = [wlkata_robot2.angle.joint1, wlkata_robot2.angle.joint2, wlkata_robot2.angle.joint3,
                       wlkata_robot2.angle.joint4, wlkata_robot2.angle.joint5, wlkata_robot2.angle.joint6]
            task_pos = [wlkata_robot2.cartesian.x, wlkata_robot2.cartesian.y, wlkata_robot2.cartesian.z,
                        wlkata_robot2.cartesian.roll, wlkata_robot2.cartesian.pitch, wlkata_robot2.cartesian.yaw]

    print("[robot_ctr] Escape the program\n")

def loadcell():
    global weight_mode
    global weight
    global val

    import time
    import sys
    import RPi.GPIO as GPIO
    from hx711 import HX711

    weight_mode = 0    
    hx = HX711(17, 27)
    hx.set_reading_format("MSB", "MSB")
    hx.set_reference_unit(500)
    hx.reset()
    hx.tare()

    if weight_mode == 0:
        while (1):
            try:
                val = hx.get_weight(5)
                weight = float(round(val))
                hx.power_down()
                hx.power_up()
                time.sleep(1)

            except Exception as e:
                print(f"Error in loadcell: {str(e)}")
                GPIO.cleanup()

def input():
    global input_mode
    global robot_task_number

    input_mode = 0
    robot_task_number = -1

def conveyor_ctr():
    global is_ctr_on
    global input_mode
    global ctr_mode
    global robot_task_number

    is_ctr_on = True
    ctr_mode = 0
    i = 0
    speed = 2000
    robot_task_number = -1
    count = 0

    while is_ctr_on:
        sleep(0.1)
        ctr_mode = robot_task_number
        if wlkata_robot3.get_status().state == 'Idle'  and input_mode == 0:
            if ctr_mode == 2 or ctr_mode == 16:
                if count == 0:
                    wlkata_robot3.set_slider_posi(50000, speed=speed)
                    count = 1

            elif ctr_mode == 119:
                ctr_mode = -1
                count = 0
                print("Emergency Stop!")
            else:
                ctr_mode = -1
        


############################ Main program (multi-threding control) ############################
if __name__ == '__main__':
    t1 = threading.Thread(target=server)
    t2 = threading.Thread(target=robot_ctr1)
    t3 = threading.Thread(target=robot_ctr2)
    t4 = threading.Thread(target=beckhoff_comm)
    t5 = threading.Thread(target=loadcell)
    t6 = threading.Thread(target=input)
    t7 = threading.Thread(target=conveyor_ctr)

    t1.start()
    t2.start()
    t3.start()
    t4.start()
    t5.start()
    t6.start()
    t7.start()

    print("Main Thread")