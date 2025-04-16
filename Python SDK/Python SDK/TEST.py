'''
JOHO舵机测试
'''
# 添加uservo.py的系统路径
from os import system
import sys
from turtle import delay
sys.path.append("./src")

import time
import serial
import struct
import sys
from uart_servo import UartServoManager
from data_table import *
# 参数配置
SERVO_PORT_NAME =  'COM6' 	# 舵机串口号
SERVO_BAUDRATE = 115200 	# 舵机的波特率
SERVO_ID = 1 				# 舵机ID 
# 初始化串口
uart = serial.Serial(port=SERVO_PORT_NAME, baudrate=SERVO_BAUDRATE,\
					 parity=serial.PARITY_NONE, stopbits=1,\
					 bytesize=8,timeout=0)
# 创建舵机对象
uservo = UartServoManager(uart, servo_id_list=[SERVO_ID])
# position = 0
# runtime_ms = 1500
# uservo.async_set_position(SERVO_ID, position, runtime_ms)
# # # 这里写其他舵机的 async_set_position, 举例子：
# # SERVO_ID = 2
# # uservo.async_set_position(SERVO_ID, position, runtime_ms)

# # 统一开始执行
# uservo.async_action()

# # 等待所有舵机执行完成动作
# uservo.wait_all()


# #等待
# time.sleep(2)

# # 读取数据
# position = uservo.read_data_by_name(SERVO_ID, "CURRENT_POSITION")
# print(f"当前位置:  {position}")
# time.sleep(0.1)
# SERVO_ID = 1 
# uservo.async_set_position(SERVO_ID, 4090, 500)
# # 统一开始执行
# uservo.async_action()

# # 等待所有舵机执行完成动作
# uservo.wait_all()
# time.sleep(1)
# # 读取数据
# position = uservo.read_data_by_name(SERVO_ID, "CURRENT_POSITION")
# print(f"当前位置:  {position}")
erro = [0,0,0,0,0,0,0,0,0,0]
runtime_ms = 500
while(1):
    position = 0
    if SERVO_ID > 10:
        SERVO_ID = 1
        #system('cls')
    uservo.async_set_position(SERVO_ID, position, runtime_ms)
    uservo.async_action()
    uservo.wait_all()
    time.sleep(1.5)
    # 读取数据
    position = uservo.read_data_by_name(SERVO_ID, "CURRENT_POSITION")
    if position is None :
        print(f"舵机:{SERVO_ID} 无法读取")
        SERVO_ID += 1
        continue
    if (position-0)>11:
        erro[SERVO_ID-1] += 1

    print(f"舵机:{SERVO_ID} 当前位置:  {position},错误：{erro[SERVO_ID-1]}次")
    time.sleep(0.2)

    position = 4095
    uservo.async_set_position(SERVO_ID, position, runtime_ms)
    uservo.async_action()
    uservo.wait_all()
    time.sleep(1.5)
    # 读取数据
    position = uservo.read_data_by_name(SERVO_ID, "CURRENT_POSITION")
    if(position-4095)>11:
        erro[SERVO_ID-1] += 1
    print(f"舵机:{SERVO_ID} 当前位置:  {position},错误：{erro[SERVO_ID-1]}次")
    time.sleep(0.2)
    uservo.wait_all()
    SERVO_ID += 1