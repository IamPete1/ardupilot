#!/usr/bin/env python
'''
example rover for JSON backend using pybullet
based on racecar example from pybullet
'''

import os, inspect, sys

import pybullet as p
import pybullet_data
import socket
import struct
import json
import numpy as np

GRAVITY_MSS = 9.80665
TIME_STEP = 1/400

from pyrobolearn.simulators import Bullet
from pyrobolearn.worlds import BasicWorld
from pyrobolearn.robots import Quadcopter

# Create simulator
sim = Bullet()

# create world
world = BasicWorld(sim)

# create robot
robot = Quadcopter(sim)

# use pymavlink for ArduPilot convention transformations
from pymavlink.rotmat import Vector3, Matrix3
from pymavlink.quaternion import Quaternion

import time

cid = p.connect(p.SHARED_MEMORY)
if (cid < 0):
  p.connect(p.GUI)

p.resetSimulation()
p.setGravity(0, 0, -GRAVITY_MSS)

p.loadSDF(os.path.join(pybullet_data.getDataPath(), "stadium.sdf"))

quad = p.loadURDF("quadrotor.urdf")

p.setTimeStep(TIME_STEP)
time_now = 0
last_velocity = None

name_to_index = {}

def quaternion_to_AP(quaternion):
    '''convert pybullet quaternion to ArduPilot quaternion'''
    return Quaternion([quaternion[3], quaternion[0], -quaternion[1], -quaternion[2]])

pos,quaternion = p.getBasePositionAndOrientation(quad)

# hack for testing angles
# p.resetBasePositionAndOrientation(quad, (0,0,3.0), (0.9,0.2,0.4,1.0))

def init():
  global time_now
  time_now = 0
  p.resetBasePositionAndOrientation(quad, (0,0,0), (0.0,0.0,0.0,1.0))

def constrain(v,min_v,max_v):
    '''constrain a value'''
    if v < min_v:
        v = min_v
    if v > max_v:
        v = max_v
    return v

def apply_quadcopter_forces(quad, motors):
    global cid
    thrust = [ constrain((m-1100.0)/800.0,0,1) for m in motors ]
    arm_length = 0.1
    motor_points = np.array([[1,-1,0], [-1,1,0], [1,1,0], [-1,-1,0]]) * arm_length
    yaw_mul = [-1,-1,1,1]
    yaw_scale = 0.1
    yaw_torque = 0.0
    thrust_scale = 5.0
    pos,quaternion = p.getBasePositionAndOrientation(quad)
    quaternion = quaternion_to_AP(quaternion)
    dcm = quaternion.dcm
    body_dir = Vector3(0,0,-1)
    earth_dir = dcm * body_dir
    for i in range(4):
        pt = motor_points[i]
        force = earth_dir * thrust[i] * thrust_scale
        yaw_torque += yaw_mul[i] * yaw_scale * thrust[i]
        p.applyExternalForce(quad, -1, [force.x, force.y, -force.z], pt, p.LINK_FRAME)
    p.applyExternalTorque(quad, -1, [0,0,yaw_torque], p.LINK_FRAME)


def physics_step(pwm_in):
  # add vertical motors
  apply_quadcopter_forces(quad, pwm_in[0:4])

  p.stepSimulation()

  # have to keep track of time manually for some reason
  global time_now
  time_now += TIME_STEP

  # get the position of the vehicle to return to AP
  pos,quaternion = p.getBasePositionAndOrientation(quad)
  quaternion = quaternion_to_AP(quaternion)
  euler = quaternion.euler

  velo,gyro = p.getBaseVelocity(quad)

  # convert to ArduPilot conventions
  gyro = (gyro[0], -gyro[1], -gyro[2])
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
  last_velocity = velocity

  # add in gravity in earth frame
  accel.z -= GRAVITY_MSS

  # convert accel to body frame
  accel = dcm.transposed() * accel

  # convert to a tuple
  accel = (accel.x, accel.y, accel.z)
  euler = quaternion.euler
  euler = (euler[0],euler[1],euler[2])

  return time_now,gyro,accel,pos,euler,velo

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', 9002))

last_SITL_frame = -1
connected = False
frame_count = 0
frame_time = time.time()
print_frame_count = 1000

print("num: ", p.getNumLinks(quad))
for i in range(p.getNumJoints(quad)):
    name = p.getJointInfo(quad, i)[12].decode('UTF-8')
    name_to_index[name] = i
print(name_to_index)


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
    print("{:.2f} fps, {:.2f}%% of realtime".format(print_frame_count/total_time,(print_frame_count*TIME_STEP)/total_time))
    frame_time = now

  # Time sync
  while True:
    if (time.time() - py_time) >= (TIME_STEP / speedup):
      break
