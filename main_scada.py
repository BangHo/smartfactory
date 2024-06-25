from __future__ import print_function
from http import server
from http.client import responses
from operator import truediv
from art import *
import os
import logging

import grpc
import data_pb2
import data_pb2_grpc
import time
from datetime import datetime
import signal


channel = grpc.insecure_channel("192.168.0.20:50052")   # rp1
channel2 = grpc.insecure_channel("192.168.0.23:50052")  # DeskTop
channel3 = grpc.insecure_channel("192.168.0.10:50052")  # rp2


stub = data_pb2_grpc.DataStub(channel)
stub2 = data_pb2_grpc.DataStub(channel2)
stub3 = data_pb2_grpc.DataStub(channel3)


def get_server_status():
    response = stub.get_server_status(data_pb2.Empty())
    response2 = stub2.get_server_status(data_pb2.Empty())
    response3 = stub3.get_server_status(data_pb2.Empty())
    return response, response2, response3

def get_status():
    response = stub.get_status(data_pb2.Empty())
    #print("string value:", response.strVal, "integer value: ", response.intVal, "float value:", response.floatVal, "double value:", response.doubleVal)        
    return response

def get_joint_angle():
    response = stub3.get_joint_angle(data_pb2.Empty())
    return response

def get_task_position():
    response = stub3.get_task_position(data_pb2.Empty())
    return response

def set_robot_task(intVal):
    response = stub2.set_robot_task(data_pb2.IntDataRequest(intVal = intVal))
    response2 = stub3.set_robot_task(data_pb2.IntDataRequest(intVal = intVal))
    return response, response2


def get_sensor_env():
    response = stub.get_sensor_env(data_pb2.Empty())
    return response

def get_sensor_env_1():
    response = stub.get_sensor_env_1(data_pb2.Empty())
    return response

def get_sensor_env_2():
    response = stub.get_sensor_env_2(data_pb2.Empty())
    return response

def homing():
    response = stub.homing(data_pb2.Empty())
    return response

def move_zero():
    response = stub.move_zero(data_pb2.Empty())
    return response

def move_joint():
    response = stub.move_joint(data_pb2.Empty())
    return response

def get_sensor_dist():
    response = stub3.get_sensor_dist(data_pb2.Empty())
    return response

def get_rfid():
    response = stub.get_rfid(data_pb2.Empty())
    return response

def get_weight_value():
    response = stub3.get_weight_value(data_pb2.Empty())
    return response

def get_camera_value():
    response = stub2.get_camera_value(data_pb2.Empty())
    return response

def terminate_server():
    response = stub.terminate_server(data_pb2.Empty())
    return response

def terminate_program():
    response = stub.terminate_program(data_pb2.Empty())
    return response


def handler(signum, handler): # 동작 입력부
    print("\n동작 입력\n")

    print("-------------------------------------------------")
    print("|   0: 로봇 호밍 |  1: 조인트 0 |  2: 전체 동작 |")
    print("|  11: 로봇 1번  | 12: 로봇 2번 | 13: 로봇 3번  |")
    print("|  14: 로봇 4번  | 15: 벨트 1번 | 16: 벨트 2번  |")
    print("| 119: 비상 정지 |\t        |\t        |")
    print("-------------------------------------------------\n")

    robot_task = int(input("로봇 동작: "))
    # 0: 로봇 호밍, 1 로봇 Set_Joint_State 0, 2: 전체 동작(일괄)
    # 11: 로봇 1번, 12: 로봇 2번, 13: 로봇 3번, 14: 로봇 4번, 15: 컨베이어 1번, 16: 컨베이어 2번
    # 119: 비상 정지
    print()
    temp = list(str(set_robot_task(robot_task)).split(" "))
    print_result = temp[1:5]
    print(" ".join(print_result).strip("\n\",")) # 동작 수행 결과 출력

    if robot_task == 119:                        # EMERGENCY STOP
        os.system("cls")
        print("\n")
        tprint("EMERGENCY STOP")

signal.signal(signal.SIGINT, handler)   # 시스템 인터럽트 처리 (입력 모드 전환)

os.system("cls")
print("#################################################################################\n")
tprint("\t\t\t\tSCADA", space = 3)
print("#################################################################################\n\n\n")

print("\t\t\t\tSystem Online\n\n\n")
print("사용법")
print("로봇 동작 입력 모드: Ctrl + C 를 누르세요.")
print("프로그램 나가기: 로봇 동작 입력 모드에서 아무 알파벳이나 입력해주세요.\n\n")

# 서버 상태 출력

server_status_list = []
for i in range(len(str(get_server_status()).split(" "))):
    if i % 2 == 1:
        server_status_list.append((str(get_server_status()).split(" "))[i].strip("\"\n,)"))

for i in range(len(server_status_list)):
    print("Server {0} : {1}".format(i + 1, server_status_list[i]))

print()

# camera_temp = 0     # 카메라 임시 변수
# cur_camera = 0

count_red = 0       # 양품 갯수
count_other = 0     # 불량품 갯수
count_box = 0       # 작업 BOX 변수

# 현재 상태 출력

while(1):
    time.sleep(1) # 1초 단위
    temp = float((str(object=get_sensor_env_1()).split(" ")[1].strip("\n")))            # 온도
    humidity = float((str(object=get_sensor_env_2()).split(" ")[1].strip("\n")))        # 습도
    rfid = str(get_rfid()).split(" ")[1].strip("\n\"")                                  # RFID
    dist = float((str(object=get_sensor_dist()).split(" ")[1].strip("\n")))             # 근접 거리 센서
    weight_value = float((str(object=get_weight_value()).split(" ")[1].strip("\n")))    # 무게
    # camera_value = get_camera_value()
    # if len(str(camera_value).split(" ")) == 2:
    #     camera_value = int(str(get_camera_value()).split(" ")[1].strip("\n"))
    # else:
    #     camera_value = 0
    # camera_temp = camera_value

    # if camera_temp != camera_value and camera_temp == 0:
    #     camre_temp = camera_value

    if round(weight_value) > 1: # 무게 반올림
        weight = 3.0
    else:
        weight = 0.0

    # if camera_temp == 1:
    #     count_red += 1
    # elif camera_temp == 2:
    #     count_other += 1

    now = datetime.now()
    print(now.strftime('\n%Y-%m-%d %H:%M:%S\n'))

    print("온도는 {0}℃ 입니다.".format(temp))
    print("습도는 {0}% 입니다.".format(humidity))
    print("저울 무게는 {0}g 입니다.".format(weight))

    if rfid == "None":
        print("센서 부근에 RFID가 없습니다.")
    else: 
        print("RFID ID는 {0} 입니다.".format(rfid))
        count_box += 1
        if rfid == "270054582710":
            print("\n토마토1\n")
            print("수확일: 2023-12-11")
            print("입고일: 2023-12-13")
        elif rfid == "888412432767":
            print("\n토마토2\n")
            print("수확일: 2023-12-11")
            print("입고일: 2023-12-13")
        elif rfid == "751241914671":
            print("\n토마토3\n")
            print("수확일: 2023-12-12")
            print("입고일: 2023-12-13")
        elif rfid == "201603541430":
            print("\n토마토4\n")
            print("수확일: 2023-12-12")
            print("입고일: 2023-12-13")
        elif rfid == "338639841790":
            print("\n토마토5\n")
            print("수확일: 2023-12-13")
            print("입고일: 2023-12-13")
        elif rfid == "888278215031":
            print("\n토마토6\n")
            print("수확일: 2023-12-13")
            print("입고일: 2023-12-13")
            
    print("현재 포장 적재량은 {0}BOX 입니다.".format(count_box))

    if dist < 50:
        print("\n[경고] 작업자와 로봇 충돌 위험!\n")
    

    

