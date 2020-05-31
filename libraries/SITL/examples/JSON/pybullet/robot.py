#!/usr/bin/env python
'''
example rover for JSON backend using pybullet
based on racecar example from pybullet
'''

import os, inspect, sys

import socket
import struct
import json
import math

from pyrobolearn.simulators import Bullet
from pyrobolearn.worlds import BasicWorld

# use pymavlink for ArduPilot convention transformations
from pymavlink.rotmat import Vector3, Matrix3
from pymavlink.quaternion import Quaternion
from pyrobolearn.utils.transformation import get_rpy_from_quaternion

import time

import argparse
from math import degrees, radians

parser = argparse.ArgumentParser(description="pybullet robot")
parser.add_argument("--vehicle", required=True, choices=['quad', 'racecar', 'iris', 'opendog'], help="vehicle type")
parser.add_argument("--fps", type=float, default=1000.0, help="physics frame rate")

args = parser.parse_args()

TIME_STEP = 1.0 / args.fps
GRAVITY_MSS = 9.80665

# Create simulator
sim = Bullet()

# create world
world = BasicWorld(sim)

def control_quad(pwm):
    '''control quadcopter'''
    motor_dir = [ 1, 1, -1, -1 ]
    motor_order = [ 0, 1, 2, 3 ]

    motors = pwm[0:4]
    motor_speed = [ 0 ] * 4
    for m in range(len(motors)):
        motor_speed[motor_order[m]] = constrain(motors[m] - 1000.0, 0, 1000) * motor_dir[motor_order[m]]

    robot.set_propeller_velocities(motor_speed)

def control_racecar(pwm):
    '''control racecar'''
    steer_max = 45.0
    throttle_max = 200.0
    steering = constrain((pwm[0] - 1500.0)/500.0, -1, 1) * math.radians(steer_max) * -1
    throttle = constrain((pwm[2] - 1500.0)/500.0, -1, 1) * throttle_max

    robot.steer(steering)
    robot.drive(throttle)

last_angles = [0.0] * 12

def control_joints(pwm):
    '''control a joint based bot'''
    global last_angles
    max_angle = radians(90)
    joint_speed = radians(30)
    pwm = pwm[0:len(robot.joints)]
    angles = [ constrain((v-1500.0)/500.0, -1, 1) * max_angle for v in pwm ]
    current = last_angles
    max_change = joint_speed * TIME_STEP
    for i in range(len(angles)):
        angles[i] = constrain(angles[i], current[i]-max_change, current[i]+max_change)
    robot.set_joint_positions(angles, robot.joints)
    last_angles = angles

if args.vehicle == 'iris':
    from pyrobolearn.robots import Quadcopter
    robot = Quadcopter(sim, urdf="models/iris/iris.urdf")
    control_pwm = control_quad
elif args.vehicle == 'racecar':
    from pyrobolearn.robots import F10Racecar
    robot = F10Racecar(sim)
    control_pwm = control_racecar
elif args.vehicle == 'opendog':
    from pyrobolearn.robots import OpenDog
    robot = OpenDog(sim, urdf="models/opendog/opendog.urdf")
    control_pwm = control_joints
else:
    raise Exception("Bad vehicle")


sim.set_time_step(TIME_STEP)

time_now = 0
last_velocity = None

def quaternion_to_AP(quaternion):
    '''convert pybullet quaternion to ArduPilot quaternion'''
    return Quaternion([quaternion[3], quaternion[0], -quaternion[1], -quaternion[2]])

def vector_to_AP(vec):
    '''convert pybullet vector tuple to ArduPilot Vector3'''
    return Vector3(vec[0], -vec[1], -vec[2])

def quaternion_from_AP(q):
    '''convert ArduPilot quaternion to pybullet quaternion'''
    return [q.q[1], -q.q[2], -q.q[3], q.q[0]]

def to_tuple(vec3):
    '''convert a Vector3 to a tuple'''
    return (vec3.x, vec3.y, vec3.z)

def init():
  global time_now
  time_now = 0
  robot.position = [0,0,0]
  robot.orientation = [0,0,0,1]

def constrain(v,min_v,max_v):
    '''constrain a value'''
    if v < min_v:
        v = min_v
    if v > max_v:
        v = max_v
    return v

print(robot.joints)

#robot.position = [ 0, 0, 2]
#robot.orientation = quaternion_from_AP(Quaternion([math.radians(0), math.radians(0), math.radians(50)]))

def physics_step(pwm_in):

  control_pwm(pwm_in)

  world.step(sleep_dt=0)

  global time_now
  time_now += TIME_STEP

  # get the position orientation and velocity
  quaternion = quaternion_to_AP(robot.orientation)
  roll, pitch, yaw = quaternion.euler
  velocity = vector_to_AP(robot.linear_velocity)
  position = vector_to_AP(robot.position)

  # get ArduPilot DCM matrix (rotation matrix)
  dcm = quaternion.dcm

  # get gyro vector in body frame
  gyro = dcm.transposed() * vector_to_AP(robot.angular_velocity)
  
  # calculate acceleration
  global last_velocity
  if last_velocity is None:
      last_velocity = velocity

  accel = (velocity - last_velocity) * (1.0 / TIME_STEP)
  last_velocity = velocity

  # add in gravity in earth frame
  accel.z -= GRAVITY_MSS

  # convert accel to body frame
  accel = dcm.transposed() * accel

  # convert to tuples
  accel = to_tuple(accel)
  gyro = to_tuple(gyro)
  position = to_tuple(position)
  velocity = to_tuple(velocity)
  euler = (roll, pitch, yaw)

  return time_now,gyro,accel,position,euler,velocity

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', 9002))

last_SITL_frame = -1
connected = False
frame_count = 0
frame_time = time.time()
print_frame_count = 1000

while True:

  py_time = time.time()

  data,address = sock.recvfrom(100)

  if len(data) != 4 + 16*2:
    continue

  decoded = struct.unpack('IHHHHHHHHHHHHHHHH',data)

  SITL_frame = decoded[0]
  pwm = decoded[1:17]

  # Check if the fame is in expected order
  if SITL_frame < last_SITL_frame:
    # Controller has reset, reset physics also
    init()
    print('Controller reset')
  elif SITL_frame == last_SITL_frame:
    # duplicate frame, skip
    print('Duplicate input frame')
    continue
  elif SITL_frame != last_SITL_frame + 1 and connected:
    print('Missed {0} input frames'.format(SITL_frame - last_SITL_frame - 1))
  last_SITL_frame = SITL_frame

  if not connected:
    connected = True
    print('Connected to {0}'.format(str(address)))
  frame_count += 1

  # physics time step
  phys_time,gyro,accel,pos,euler,velo = physics_step(pwm)

  # build JSON format
  IMU_fmt = {
    "gyro" : gyro,
    "accel_body" : accel
  }
  JSON_fmt = {
    "timestamp" : phys_time,
    "imu" : IMU_fmt,
    "position" : pos,
    "attitude" : euler,
    "velocity" : velo
  }
  JSON_string = "\n" + json.dumps(JSON_fmt,separators=(',', ':')) + "\n"

  # Send to AP
  sock.sendto(bytes(JSON_string,"ascii"), address)

  # Track frame rate
  if frame_count % print_frame_count == 0:
    now = time.time()
    total_time = now - frame_time
    print("%.2f fps T=%.3f dt=%.3f" % (print_frame_count/total_time, phys_time, total_time))
    frame_time = now
