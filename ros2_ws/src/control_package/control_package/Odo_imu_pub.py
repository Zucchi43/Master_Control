'''
Filename: c:\Users\Gabri\OneDrive\Documentos\Mestrado-USP-2022\ProjetoROTA2030\src\control_package\control_package\Odo_imu_pub.py
Path: c:\Users\Gabri\OneDrive\Documentos\Mestrado-USP-2022\ProjetoROTA2030\src\control_package\control_package
Created Date: Saturday, April 6th 2024, 3:24:37 pm
Author: Gabriel Zucchi
Updated: 17, May 2024

--With covariances,time and publshing imu, odom and speed_steer_feedback

Updated: 28, May 2024

--Added x and y pose to Odom msg.
--refactored time calculation on yaw calculation for IMU
--Changed the mapping function for "IMU" sensor (VDC2_BS) to hex_to_value



TO DO: Refactor the code to objected oriented!!

Copyright (c) 2024 Your Company
'''
import can
import time
import rclpy
import math
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion

pi = 3.1416
speed = 0
steer = 0
yaw = 0
Acc_X = 0
Acc_Y = 0
steer_angle = 0
tempo = time.time()
yaw_rate = 0.0
new_yaw = 0.0
oldx = 0.0
oldy =0.0
olddt = time.time()

def remap(old_val, old_min, old_max, new_min, new_max):
    return (new_max - new_min)*(old_val - old_min) / (old_max - old_min) + new_min

#THIS FUNTION IS USED TO GET THE VALUES FROM CAN AND CONVERT ACCORDING TO MERCEDES DATA THAT WAS PROVIDED
def hex_to_value(val, factor,offset):
    return val*factor + offset


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

def remap_turn(turn_hex):
    table={'0x20':0,
           '0x21':1,
           '0x22':2,
           '0x23':3,
           '0x1F':-1,
           '0x1E':-2,
           '0x1D':-3}
    if(turn_hex == '0x1d'): return -3
    elif(turn_hex == '0x20'): return 0
    elif(turn_hex == '0x21'): return 1
    elif(turn_hex == '0x22'): return 2
    elif(turn_hex == '0x23'): return 3
    elif(turn_hex == '0x1f'): return -1
    elif(turn_hex == '0x1e'): return -2
    return 0

def byte_to_value_steer(data):
    bytes_resto = hex( (data[1] << 8) | data[0])
    bytes_turns = hex(data[2])
    bytes_pi = hex( (data[5] << 8) | data[4])
    resto_rad = remap(int(bytes_resto,16),int('7185',16),int('8A10',16),-pi,pi)
    converted_data = remap_turn(bytes_turns) *(2*pi)  + resto_rad
    return float(converted_data)

def byte_to_value_speed(data):
    byte_speed_dec = hex(data[7])
    byte_speed_frac = hex(data[6])
    frac_dec = remap(int(byte_speed_frac,16),0,int('80',16),0,0.5)
    converted_data = int(byte_speed_dec,16) + frac_dec
    converted_data = converted_data /3.6
    return float(converted_data)

def imu_read(data, current_yaw):
    global tempo
    steer_raw = data[1] << 8 | data[0]
    steer_raw = remap(steer_raw,0,65535,-31.374,31.375)

    yaw_data = data[4] << 8 | data[3]#RATE rad/s
    #yaw_data = remap(yaw_data,0,65535,-3.92,3.92)
    if(yaw_data == 65535): yaw_data =0 #INITIAL VALUE
    else: yaw_data = hex_to_value(yaw_data,0.0001220703125,-3.92)
    
    dt = time.time() - tempo
    if(abs(yaw_data) > 3.8 or abs(yaw_data) <= 0.0005): yaw_data = 0
    new_yaw = current_yaw + float(yaw_data * dt) #rad
    
    accy = data[6] << 8 | data[5]
    #accy = remap(accy,0,65535,-15.687,15.687)
    if(accy == 65535): accy =0 #INITIAL VALUE
    else: accy = hex_to_value(accy,0.00048828125,-15.687)#m/s²
    accx = data[7]
    if(accx == 255 ): accx =0 #INITIAL VALUE
    #accx = remap(accx,0,255,-12.5,12.5)
    else: accx = hex_to_value(accx,0.1,-12.5)#m/s²

    tempo = time.time()

    return float(steer_raw), float(new_yaw),float(yaw_data) , float(accx), float(accy)

def update(msg):
    global speed, steer, yaw, tempo, yaw_rate, Acc_X, Acc_Y, steer_angle,olddt
    #current_time = time.time()
    if(msg.arbitration_id == int("0xCFE6C17",0)):
        speed = byte_to_value_speed(msg.data)
        pub_ros2_twist(1)
    elif(msg.arbitration_id == int("0x8F01D13",0)):
        steer = byte_to_value_steer(msg.data)
        pub_ros2_twist(0)
    elif(msg.arbitration_id == int("0x8F0090B",0)):
        (steer_angle, yaw, yaw_rate, Acc_X, Acc_Y) = imu_read(msg.data, yaw)
        pub_ros2_odom_imu()

    return 0

def connect_canbus():
    global bus, tempo
    filters = [
        {"can_id": 0xCFE6C17,"can_mask":0x1FFFFFFF, "extended": True},#SPEED
        {"can_id": 0x8F01D13, "can_mask": 0x1FFFFFFF, "extended": True},#STEER
        {"can_id": 0x8F0090B, "can_mask": 0x1FFFFFFF, "extended": True}, #Yaw, Accel X and Accel Y
    ]
    bus = can.interface.Bus(channel="can0", bustype="socketcan", can_filters=filters)
    tempo = time.time()
    __notifier = can.Notifier(bus, [update], 0)

def pub_ros2_odom_imu():
    global node, oldx, oldy,olddt
    odom_msg = Odometry()
    odom_msg.header.stamp = node.get_clock().now().to_msg()
    odom_msg.header.frame_id = "odom"
    odom_msg.child_frame_id = "base_footprint"
    odom_msg.twist.twist.linear.x = float(speed/3.6)#m/s
    dt = time.time() - olddt
    odom_msg.pose.pose.position.x = oldx + float( odom_msg.twist.twist.linear.x * math.cos(yaw) )*dt
    odom_msg.pose.pose.position.y = oldy + float( odom_msg.twist.twist.linear.x * math.sin(yaw) )*dt
    (odom_msg.pose.covariance[0],odom_msg.pose.covariance[7],odom_msg.pose.covariance[14],odom_msg.pose.covariance[21],odom_msg.pose.covariance[28],odom_msg.pose.covariance[35]) =(0.00001,0.00001,1000000000000.0,1000000000000.0,1000000000000.0,0.001)
    (odom_msg.twist.covariance[0],odom_msg.twist.covariance[7],odom_msg.twist.covariance[14],odom_msg.twist.covariance[21],odom_msg.twist.covariance[28],odom_msg.twist.covariance[35]) =(0.00001,0.00001,1000000000000.0,1000000000000.0,1000000000000.0,0.001)

    imu_msg = Imu()
    imu_msg.header.stamp = node.get_clock().now().to_msg()
    imu_msg.header.frame_id = "base_footprint"
    quart = quaternion_from_euler(0,0,yaw)
    (imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w ) = (float(quart[0]), float(quart[1]), float(quart[2]), float(quart[3]))
    (imu_msg.angular_velocity.x,imu_msg.angular_velocity.y,imu_msg.angular_velocity.z) = (0.0, 0.0, float(yaw_rate))
    (imu_msg.linear_acceleration.x,imu_msg.linear_acceleration.y,imu_msg.linear_acceleration.z) = (float(Acc_X), float(Acc_Y), 0.0)
    imu_msg.angular_velocity_covariance = [0.00000004, 0, 0, 0, 0.00000004, 0, 0, 0, 0.00000004]
    imu_msg.linear_acceleration_covariance = [0.000289,0,0, 0,0.000289,0, 0,0,0.000289] 



    odom_pub.publish(odom_msg)
    imu_pub.publish(imu_msg)
    olddt = time.time()

def pub_ros2_twist(speed_steer_qualifier):
    twist_msg = Twist()
    if(speed_steer_qualifier == 1) : twist_msg.linear.z = 1.0
    else: twist_msg.linear.z = 0.0
    twist_msg.linear.x = float(speed)
    twist_msg.angular.z = float(steer)
    speed_steer_pub.publish(twist_msg)

def main():
    global imu_pub,odom_pub,speed_steer_pub,node
    rclpy.init()
    node = Node("publisher")
    imu_pub = node.create_publisher(Imu, 'imu',10)
    odom_pub = node.create_publisher(Odometry, 'odom',10)
    speed_steer_pub = node.create_publisher(Twist, "speed_steer_feedback",10)
    connect_canbus()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
