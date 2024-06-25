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

## modi libraries (IOT)
import modi

## RFID
import RPi.GPIO as GPIO
from mfrc522 import SimpleMFRC522
import time

## Objects
# obj = client.InputOutputTask('localhost') # beckhoff object
# wlkata_robot1 = WlkataMirobot(portname='/dev/ttyUSB0') # wlkata object
# wlkata_robot2 = WlkataMirobot(portname='/dev/ttyUSB1') # wlkata object
bundle = modi.MODI() # modi object


############################ Define variables ############################
## Variables
env = bundle.envs[0]

## Commands (pick and place, palletizing tasks)
cmd_pick_and_place = 3
cmd_palletizing = 4


## Class (grpc server)
class Data(data_pb2_grpc.DataServicer):
    def get_server_status(self, request, context):
        return data_pb2.StrDataReply(strVal=server_status)

    def get_status(self, request, context):
        return data_pb2.StrDataReply(strVal=status)

    def get_joint_angle(self, request, context):
        response = data_pb2.RepeatedFloatDataReply()
        response.floatVal.extend(jnt_ang)
        return response

    def get_task_position(self, request, context):
        response = data_pb2.RepeatedFloatDataReply()
        response.floatVal.extend(task_pos)
        return response

    def set_robot_task(self, request, context):
        global robot_task_number, robot_mode
        robot_mode = 0
        robot_task_number = request.intVal
        return data_pb2.StrDataReply(strVal="Task " + str(robot_task_number) + " Sent Done")

    def get_sensor_env(self, request, context):
        global modi_mode
        modi_mode = 0
        response = data_pb2.RepeatedFloatDataReply()
        response.floatVal.extend(env_data)
        return response
    
    def get_sensor_env_1(self, request, context):
        global modi_mode
        modi_mode = 0
        return data_pb2.FloatDataReply(floatVal=env_data_1)

    def get_sensor_env_2(self, request, context):
        global modi_mode
        modi_mode = 0
        return data_pb2.FloatDataReply(floatVal=env_data_2)

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

    def get_rfid(self, request, context):
        global rfid_mode
        rfid_mode = 0
        return data_pb2.StrDataReply(strVal=rfid_id)

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


# get IoT sensor values
def modi_comm():
    global is_comm_on
    global modi_mode
    global env_data
    global env_data_1, env_data_2

    is_comm_on = 1
    modi_mode = 0

    print("MODI ONLINE")

    while is_comm_on:
        time.sleep(1)
        if modi_mode == 0:
            env_data = [env.humidity, env.temperature, env.brightness, env.red, env.green, env.blue]
            env_data_1 = env.temperature
            env_data_2 = env.humidity

        else:
            print("no data")



    print("[modi_comm] Escape the program\n")

# control robot
def robot_ctr():
    global is_ctr_on
    global ctr_mode
    global status, jnt_ang, task_pos

    is_ctr_on = True
    ctr_mode = -1

def conveyor_ctr():
    global is_ctr_on
    is_ctr_on = True

def rfid():
    global rfid_id
    global rfid_mode
    reader = SimpleMFRC522()
    rfid_mode = 0

    print("RFID Online")

    while(1):
        if rfid_mode == 0:
            time.sleep(1)
            id = reader.read_id_no_block()
            rfid_id = str(id)

############################ Main program (multi-threding control) ############################
if __name__ == '__main__':
    t1 = threading.Thread(target=server)        # GRPC
    t2 = threading.Thread(target=modi_comm)     # MODI
    t3 = threading.Thread(target=rfid)          # RFID

    t1.start()
    t2.start()
    t3.start()

    print("Main Thread")