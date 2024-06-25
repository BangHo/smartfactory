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

# realsense lib
import cv2
import numpy as np
import pyrealsense2 as rs

import time

## Objects
obj = client.InputOutputTask('localhost') # beckhoff object
wlkata_robot1 = WlkataMirobot(portname='COM11') # wlkata object
wlkata_robot2 = WlkataMirobot(portname='COM12') # wlkata object
wlkata_robot3 = WlkataMirobot(portname='COM3') # wlkata object

# ------------------------------realsense set--------------------------
# 각 색상의 lower 및 upper 값
color_ranges_bgr = {
    "red": (np.array([150, 165, 220]), np.array([160, 170, 225])),
    "green": (np.array([180, 80, 150]), np.array([190, 90, 160])),
    "purple": (np.array([105, 135, 95]), np.array([115, 150, 120])),
    "black": (np.array([95, 110, 110]), np.array([130, 130, 130]))
}

def bgr_to_lab(bgr_value):
    lab_value = cv2.cvtColor(bgr_value, cv2.COLOR_BGR2LAB)
    return lab_value

def get_closest_color_lab(mean_lab_value):
    distances = {color: np.linalg.norm(mean_lab_value - np.mean([color_ranges_bgr[color][0], color_ranges_bgr[color][1]], axis=0)) for color in color_ranges_bgr}
    closest_color = min(distances, key=distances.get)
    return closest_color

def decide_tomato_condition(color):
    if color == "black":
        return 0
    elif color in ["green", "red"]:
        return 1
    elif color == "purple":
        return 2

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

    def get_camera_value(self, request, context):
        global camera_mode
        camera_mode = 0
        return data_pb2.IntDataReply(intVal=obj_color)

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

    env_data = 0
    env_data_1 = 0
    env_data_2 = 0

def realsense():
    global is_cam_on
    global final_color

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    pipeline.start(config)

    # ROI 좌표 (x, y, width, height)
    roi_coords = (240, 150, 90, 70)

    color_detection_count = {"red": 0, "green": 0, "purple": 0, "black": 0}

    is_cam_on = True
    try:
        while is_cam_on:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            # 색상 프레임을 numpy 배열로 변환
            color_image = np.asanyarray(color_frame.get_data())

            # ROI 설정
            x, y, w, h = roi_coords
            roi_color = color_image[y:y+h, x:x+w]

            # 색상 분류
            lab = bgr_to_lab(roi_color)

            # ROI 내부의 평균 LAB값 계산
            mean_lab = np.mean(lab, axis=(0, 1))

            # 가장 가까운 색상 찾기
            closest_color = get_closest_color_lab(mean_lab)

            # ROI 영역을 화면에 표시
            cv2.rectangle(color_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # 색상이 감지된 횟수 증가
            color_detection_count[closest_color] += 1

            # 색상이 10번 이상 감지되면 값 저장 및 결과 출력
            if color_detection_count[closest_color] >= 30:
                final_color = decide_tomato_condition(closest_color)
                print(final_color)

                # 횟수 초기화
                color_detection_count[closest_color] = 0

            # 결과를 표시
            cv2.imshow('Color Classification', color_image)

            # 'q' 키를 누르면 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # 종료 시에 파이프라인을 정리
        pipeline.stop()
        cv2.destroyAllWindows()

# control robot
def robot_ctr1():
    global is_ctr_on
    global ctr_mode
    global status, jnt_ang, task_pos
    global input_mode
    global robot_task_number
    global is_sequence
    
    is_sequence = True
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

            elif ctr_mode == 2 or ctr_mode == 11:
                # print(ctr_mode)
                if is_sequence == True:
                    wlkata_robot1.linear_interpolation(-206.4, 87, 123.4, 0, 0, 0, 3000)            #웨이포인트
                    time.sleep(0.1)     
                    wlkata_robot1.linear_interpolation(-206.4, 87, 29, 0, 0, 0, 3000)
                    time.sleep(0.1)
                    wlkata_robot1.pump_suction()                                                    #석션_on
                    time.sleep(0.1)
                    wlkata_robot1.linear_interpolation(-206.4, 87, 123.4, 0, 0, 0, 3000)
                    time.sleep(0.1)
                    wlkata_robot1.linear_interpolation(3.6, 225, 124, 0, 0, 0, 3000)
                    time.sleep(0.1)
                    wlkata_robot1.linear_interpolation(3.6, 225, 95, 0, 0, 0, 3000)
                    time.sleep(0.1)
                    wlkata_robot1.pump_off()                                                        #석션_off
                    time.sleep(0.1)
                    wlkata_robot1.linear_interpolation(3.6, 225, 124, 0, 0, 0, 3000)
                    time.sleep(0.1)
                    wlkata_robot1.linear_interpolation(-161.4, 180, 123.7, 0, 0, 0, 3000)
                    time.sleep(0.1)
                    is_sequence = False
                
            elif ctr_mode == 119:
                ctr_mode = -1
                is_sequence = True
                print("Emergency Stop!")

            else:
                pass

            status = wlkata_robot1.get_status().state
            jnt_ang = [wlkata_robot1.angle.joint1, wlkata_robot1.angle.joint2, wlkata_robot1.angle.joint3,
                       wlkata_robot1.angle.joint4, wlkata_robot1.angle.joint5, wlkata_robot1.angle.joint6]
            task_pos = [wlkata_robot1.cartesian.x, wlkata_robot1.cartesian.y, wlkata_robot1.cartesian.z,
                        wlkata_robot1.cartesian.roll, wlkata_robot1.cartesian.pitch, wlkata_robot1.cartesian.yaw]

    print("[robot_ctr] Escape the program\n")

def robot_ctr2():
    global is_ctr_on
    global ctr_mode
    global status, jnt_ang, task_pos
    global is_sequence
    global final_color
    global input_mode
    global robot_task_number
    global camera_mode
    global obj_color

    is_sequence = True
    is_ctr_on = True
    ctr_mode = -1
    input_mode = 0
    robot_task_number = -1
    final_color = -1
    camera_mode = 0
    obj_color = 0

    while is_ctr_on:
        sleep(1)
        ctr_mode = robot_task_number
        if wlkata_robot2.get_status().state == 'Idle' and input_mode == 0:
            ctr_mode = robot_task_number

            if ctr_mode == 0:
                print("homing")
                wlkata_robot2.home()
                ctr_mode = -1

            elif ctr_mode == 1:
                print("move zero")
                wlkata_robot2.go_to_zero()
                ctr_mode = -1

            elif ctr_mode == 2 or ctr_mode == 12:
                if final_color == 1:
                    wlkata_robot2.linear_interpolation(50, -215, 120.7, 0, 0, 0, 3000)              # 동작 1
                    time.sleep(0.1)
                    wlkata_robot2.linear_interpolation(50, -215, 89, 0, 0, 0, 3000)
                    time.sleep(0.1)
                    wlkata_robot2.pump_suction()                                                
                    time.sleep(0.1)
                    wlkata_robot2.linear_interpolation(50, -215, 120.7, 0, 0, 0, 3000)
                    time.sleep(0.1)
                    wlkata_robot2.linear_interpolation(164.9, 0, 175.4, 0, 0, 0, 3000)
                    time.sleep(0.1)
                    wlkata_robot2.linear_interpolation(50, 194.9, 100.1, 0, 0, 0, 3000)
                    time.sleep(0.1)
                    wlkata_robot2.linear_interpolation(50, 194.9, 85.8, 0, 0, 0, 3000)
                    time.sleep(0.1)
                    wlkata_robot2.pump_off()                                                        #석션_off
                    time.sleep(0.1)
                    wlkata_robot2.linear_interpolation(50, 194.9, 100.1, 0, 0, 0, 3000)
                    time.sleep(0.1)
                    wlkata_robot2.linear_interpolation(164.9, 0, 175.4, 0, 0, 0, 3000)
                    time.sleep(0.1)
                    wlkata_robot2.linear_interpolation(164.9, 0, 175.4, 0, 0, 0, 3000)
                    time.sleep(0.1)
                    is_sequence = True
                    obj_color = 1

                elif final_color == 2:
                    wlkata_robot2.linear_interpolation(50, -215, 120.7, 0, 0, 0, 3000)              # 동작 2
                    time.sleep(0.1)
                    wlkata_robot2.linear_interpolation(50, -215, 89, 0, 0, 0, 3000)
                    time.sleep(0.1)
                    wlkata_robot2.pump_suction()                                                
                    time.sleep(0.1)
                    wlkata_robot2.linear_interpolation(50, -215, 120.7, 0, 0, 0, 3000)
                    time.sleep(0.1)
                    wlkata_robot2.linear_interpolation(164.9, 0, 175.4, 0, 0, 0, 3000)
                    time.sleep(0.1)
                    wlkata_robot2.linear_interpolation(-85, 199.8, 155.8, 0, 0, 0, 3000)
                    time.sleep(0.1)
                    wlkata_robot2.linear_interpolation(-85, 199.8, 100, 0, 0, 0, 3000)
                    time.sleep(0.1)
                    wlkata_robot2.pump_off()                                                        #석션_off
                    time.sleep(0.1)
                    wlkata_robot2.linear_interpolation(-85, 199.8, 155.8, 0, 0, 0, 3000)
                    time.sleep(0.1)
                    wlkata_robot2.linear_interpolation(164.9, 0, 175.4, 0, 0, 0, 3000)
                    time.sleep(0.1)
                    wlkata_robot2.linear_interpolation(164.9, 0, 175.4, 0, 0, 0, 3000)
                    time.sleep(0.1)
                    is_sequence = True
                    obj_color = 2

                else:
                    pass

            # 로봇팔 비상 정지를 위한 컨트롤 모드
            elif ctr_mode == 119:
                ctr_mode = -1
                is_sequence = True
                print("Emergency Stop!")
            else:
                pass

            status = wlkata_robot2.get_status().state
            jnt_ang = [wlkata_robot2.angle.joint1, wlkata_robot2.angle.joint2, wlkata_robot2.angle.joint3,
                       wlkata_robot2.angle.joint4, wlkata_robot2.angle.joint5, wlkata_robot2.angle.joint6]
            task_pos = [wlkata_robot2.cartesian.x, wlkata_robot2.cartesian.y, wlkata_robot2.cartesian.z,
                        wlkata_robot2.cartesian.roll, wlkata_robot2.cartesian.pitch, wlkata_robot2.cartesian.yaw]

    print("[robot_ctr] Escape the program\n")

def conveyor_ctr():
    global is_ctr_on
    global input_mode
    global ctr_mode
    global robot_task_number

    is_ctr_on = True
    ctr_mode = -1
    i = 0
    speed = 2000
    robot_task_number = -1
    count = 0

    while is_ctr_on:
        sleep(0.1)
        ctr_mode = robot_task_number
        if wlkata_robot3.get_status().state == 'Idle'  and input_mode == 0:
            if ctr_mode == 2 or ctr_mode == 15:
                if count ==0:
                    wlkata_robot3.set_slider_posi(30000, speed=speed)
                    count=1

            elif ctr_mode == 119:
                ctr_mode = -1
                count = 0
                print("Emergency Stop!")
            else:
                i=0
                pass

def rfid():
    global rfid_id
    global rfid_mode

    rfid_id = 0
    rfid_mode = 0

def test():
    global input_mode
    global robot_task_number

    input_mode = 0
    robot_task_number = -1
    
    # while(1):
    #   if input_mode == 0:
    #        time.sleep(1)
    #        print(robot_task_number)

############################ Main program (multi-threding control) ############################
if __name__ == '__main__':
    t1 = threading.Thread(target=server)            # GRPC
    t2 = threading.Thread(target=robot_ctr1)
    t3 = threading.Thread(target=robot_ctr2)
    t4 = threading.Thread(target=realsense)
    t5 = threading.Thread(target=conveyor_ctr)
    t6 = threading.Thread(target=test)
    
    t1.start()
    t2.start()
    t3.start()
    t4.start()
    t5.start()                       
    t6.start()                       

    print("Main Thread")