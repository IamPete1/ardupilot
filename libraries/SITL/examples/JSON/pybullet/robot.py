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
parser.add_argument("--vehicle", required=True, choices=['quad', 'racecar', 'iris'], help="vehicle type")

args = parser.parse_args()

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

if args.vehicle == 'quad':
    from pyrobolearn.robots import Quadcopter
    robot = Quadcopter(sim, urdf="quadcopter.urdf")
    control_pwm = control_quad
elif args.vehicle == 'iris':
    from pyrobolearn.robots import Quadcopter
    robot = Quadcopter(sim, urdf="iris.urdf")
    control_pwm = control_quad
elif args.vehicle == 'racecar':
    from pyrobolearn.robots import F10Racecar
    robot = F10Racecar(sim)
    control_pwm = control_racecar
else:
    raise Exception("Bad vehicle")

GRAVITY_MSS = 9.80665
TIME_STEP = 1/1200.0

sim.set_time_step(TIME_STEP)

time_now = 0
last_velocity = None

def quaternion_to_AP(quaternion):
    '''convert pybullet quaternion to ArduPilot quaternion'''
    return Quaternion([quaternion[3], quaternion[0], -quaternion[1], -quaternion[2]])

def quaternion_from_AP(q):
    '''convert ArduPilot quaternion to pybullet quaternion'''
    return [q.q[1], -q.q[2], -q.q[3], q.q[0]]

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

#robot.position = [ 0, 0, 2]
#robot.orientation = quaternion_from_AP(Quaternion([math.radians(0), math.radians(0), math.radians(50)]))

def physics_step(pwm_in):

  control_pwm(pwm_in)

  #robot.linear_velocity = [ 0, 0, 0 ]

  world.step(sleep_dt=0)

  global time_now
  time_now += TIME_STEP

  # get the position of the vehicle to return to AP
  pos = robot.position
  quaternion = robot.orientation

  roll, pitch, yaw = get_rpy_from_quaternion(robot.orientation)
  quaternion = Quaternion([roll, -pitch, -yaw])
  euler = quaternion.euler

  velo = robot.linear_velocity

  pos = (pos[0], -pos[1], -pos[2])
  velo = (velo[0], -velo[1], -velo[2])

  velocity = Vector3(velo)

  # get ArduPilot DCM matrix (rotation matrix)
  dcm = quaternion.dcm

  # calculate acceleration
  global last_velocity
  if last_velocity is None:
      last_velocity = velocity

  accel = (velocity - last_velocity) * (1.0 / TIME_STEP)

  avel = robot.angular_velocity

  gyro = Vector3(avel[0], -avel[1], -avel[2])
  gyro = dcm.transposed() * gyro
  gyro = [gyro.x, gyro.y, gyro.z]

  last_velocity = velocity
  last_quaternion = quaternion

  # add in gravity in earth frame
  accel.z -= GRAVITY_MSS

  # convert accel to body frame
  accel = dcm.transposed() * accel

  # convert to a tuple
  accel = (accel.x, accel.y, accel.z)
  euler = (roll, -pitch, -yaw)

  return time_now,gyro,accel,pos,euler,velo

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

  if len(data) != 4 + 4 + 16*2:
    continue

  decoded = struct.unpack('IfHHHHHHHHHHHHHHHH',data)

  SITL_frame = decoded[0]
  speedup = decoded[1]
  pwm = decoded[2:18]

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
    "timestamp" : phys_time*10**-6,
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

  # Time sync
  while True:
    if (time.time() - py_time) >= (TIME_STEP / speedup):
      break
    
