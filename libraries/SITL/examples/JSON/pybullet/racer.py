#!/usr/bin/env python
'''
example rover for JSON backend using pybullet
based on racecar example from pybullet
'''

import os, inspect

import pybullet as p
import pybullet_data
import socket
import struct
import json

GRAVITY_MSS = 9.80665
TIME_STEP = 0.001

# use pymavlink for ArduPilot convention transformations
from pymavlink.rotmat import Vector3, Matrix3
from pymavlink.quaternion import Quaternion

import time

cid = p.connect(p.SHARED_MEMORY)
if (cid < 0):
  p.connect(p.GUI)

p.resetSimulation()
p.setGravity(0, 0, -GRAVITY_MSS)

#for video recording (works best on Mac and Linux, not well on Windows)
#p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "racecar.mp4")
p.loadSDF(os.path.join(pybullet_data.getDataPath(), "stadium.sdf"))

car = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "racecar/racecar.urdf"))

inactive_wheels = [3, 5, 7]
wheels = [2]

for wheel in inactive_wheels:
  p.setJointMotorControl2(car, wheel, p.VELOCITY_CONTROL, targetVelocity=0, force=0)

steering = [4, 6]

maxForceSlider = p.addUserDebugParameter("maxForce", 0, 10, 10)

p.setTimeStep(TIME_STEP)
time_now = 0
last_velocity = None

def quaternion_to_AP(quaternion):
    '''convert pybullet quaternion to ArduPilot quaternion'''
    return Quaternion([quaternion[3], quaternion[0], -quaternion[1], -quaternion[2]])

pos,quaternion = p.getBasePositionAndOrientation(car)

# hack for testing angles
# p.resetBasePositionAndOrientation(car, (0,0,3.0), (0.9,0.2,0.4,1.0))

def init():
  global time_now
  time_now = 0
  p.resetBasePositionAndOrientation(car, (0,0,0), (0.0,0.0,0.0,1.0))

def constrain(v,min_v,max_v):
    '''constrain a value'''
    if v < min_v:
        v = min_v
    if v > max_v:
        v = max_v
    return v

def apply_quadcopter_forces(car, motors):
    '''allow the car to fly!'''
    global cid
    thrust = [ constrain((m-1000.0)/1000.0,0,1) for m in motors ]
    arm_length = 0.3
    motor_points = np.array([[1,1,0], [-1,-1,0], [1,-1,0], [-1,1,0]]) * arm_length
    yaw_mul = [-1,-1,1,1]
    yaw_scale = 0.0
    yaw_torque = 0.0
    thrust_scale = 100.0
    for i in range(4):
        pt = motor_points[i]
        force = thrust[i] * thrust_scale
        yaw_torque += yaw_mul[i] * yaw_scale * thrust[i]
        p.applyExternalForce(car, -1, [0,0,-force], pt, p.LINK_FRAME)
    p.applyExternalTorque(car, 0, [0,0,yaw_torque], p.LINK_FRAME)
    

def physics_step(pwm_in):
  maxForce = p.readUserDebugParameter(maxForceSlider)

  # PWM to wheel speed and sterring angle
  targetVelocity = constrain((pwm_in[2] - 1500) / 500.0,-1,1) * 20
  steeringAngle = constrain((pwm_in[0] - 1500) / 500.0,-1, 1) * -1.0

  for wheel in wheels:
    p.setJointMotorControl2(car,
                            wheel,
                            p.VELOCITY_CONTROL,
                            targetVelocity=targetVelocity,
                            force=maxForce)

  for steer in steering:
    p.setJointMotorControl2(car, steer, p.POSITION_CONTROL, targetPosition=steeringAngle)

  p.stepSimulation()

  # have to keep track of time manually for some reason
  global time_now
  time_now += TIME_STEP

  # get the position of the vehicle to return to AP
  pos,quaternion = p.getBasePositionAndOrientation(car)
  quaternion = quaternion_to_AP(quaternion)
  euler = quaternion.euler

  velo,gyro = p.getBaseVelocity(car)

  # convert to ArduPilot conventions
  gyro = (gyro[0], -gyro[1], -gyro[2])
  pos = (pos[0], -pos[1], -pos[2])
  velo = (velo[0], -velo[1], -velo[2])

  velocity = Vector3(velo)
  position = Vector3(pos)

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

pwm = [1500,1500]

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
    print('Connected to %s', str(address))
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
    time.sleep(0.0001)
