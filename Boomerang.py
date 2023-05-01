import numpy as np
from numpy import pi
import matplotlib.pyplot as plt
import math
import matplotlib.animation as animation
from IPython import display

initX, initY = 0, 0
targetX, targetY = 20, 20
currentHeading = 0
targetHeading = 270
tolerance = 0.1
Kp_lin = 15
Kp_turn = 6.5
M_PI = 3.14159
minError = 1
canReverse = False

numOfFrames = 120
dt = 50 

noPose = False

currentPos = [initX, initY]
targetPos = [targetX, targetY]

def find_min_angle (targetHeading, currentHeading):
  turnAngle = targetHeading - currentHeading
  if turnAngle > 180 or turnAngle < -180 :
    turnAngle = -1 * np.sign(turnAngle) * (360 - abs(turnAngle))
  return turnAngle

def sgn (num):
  if num >= 0:
    return 1
  else:
    return -1
  
def get_angular_error(point_x, point_y):
  x = point_x
  y = point_y
  x -= currentPos[0]
  y -= currentPos[1]
  delta_theta = math.atan2(point_y, point_x) - (currentHeading * M_PI / 180)
  while math.fabs(delta_theta) > M_PI:
    delta_theta -= 2 * M_PI * delta_theta / math.fabs(delta_theta)
  return delta_theta
  # if delta_theta > M_PI:
  #   delta_theta -= 2 * M_PI
  # elif delta_theta < -M_PI:
  #   delta_theta += 2 * M_PI
  # return delta_theta

def boomerang(currentPos, currentHeading, targetPos, targetHeading, Kp_lin, Kp_turn):
  currentX, currentY = currentPos[0], currentPos[1]
  targetX, targetY = targetPos[0], targetPos[1]
  noPose = targetHeading > 360
  
  if noPose:
    carrot_point_x = targetX
    carrot_point_y = targetY
  else:
    h = math.sqrt((targetX-currentX)**2 + (targetY-currentY)**2)
    at = targetHeading * M_PI / 180
    carrot_point_x = targetX - h * (math.sin(at)) * 0.8
    carrot_point_y = targetY - h * (math.cos(at)) * 0.8
    print(f"carrot point x: {carrot_point_x}")
    print(f"carrot point y: {carrot_point_y}")

  linear_error = math.sqrt((targetX-currentX)**2 + (targetY-currentY)**2)
  angular_error = get_angular_error(carrot_point_x, carrot_point_y)

  linear_speed = linear_error * Kp_lin
  angular_speed = angular_error * Kp_turn

  if linear_error < minError:
    canReverse = True
    if noPose:
      angular_speed = 0
    else:
      poseError = (targetHeading * M_PI / 180) - (currentHeading * M_PI / 180)
      while (math.fabs(poseError) > M_PI):
        poseError -= 2 * M_PI * poseError / math.fabs(poseError)
        angular_speed = poseError * Kp_turn
      linear_speed *= math.cos(angular_error)
  else:
    if math.fabs(angular_error) > (2 * M_PI) and canReverse:
      angular_error = angular_error - (angular_error / math.fabs(angular_error)) * M_PI
      linear_speed = -linear_speed

    print("inside first if statemtn")
    angular_speed = angular_error * Kp_turn

  overturn = math.fabs(angular_speed) + math.fabs(linear_speed) - 100
  if overturn > 0:
    print("in overturn")
    if linear_speed > 0:
      linear_speed -= overturn
    else:
      linear_speed -= -overturn

  print(f"lin error: {linear_error}")
  print(f"angular error: {angular_error * 180 / M_PI}")
  print(f"H: {h}")
  print(f"Overturn: {overturn}")




  return linear_speed, angular_speed

def new_boomerang(currentPos, currentHeading, targetPos, targetHeading, Kp_lin, Kp_turn):
  currentX, currentY = currentPos[0], currentPos[1]
  targetX, targetY = targetPos[0], targetPos[1]
  noPose = targetHeading > 360
  
  if noPose:
    carrot_point_x = targetX
    carrot_point_y = targetY
  else:
    h = math.sqrt((targetX-currentX)**2 + (targetY-currentY)**2)
    at = targetHeading * M_PI / 180
    carrot_point_x = targetX - h * (math.sin(at)) * 0.8
    carrot_point_y = targetY - h * (math.cos(at)) * 0.8
    print(f"carrot point x: {carrot_point_x}")
    print(f"carrot point y: {carrot_point_y}")


  print(f"current angle: {(currentHeading)}")
  
  absTargetAngle = math.atan2 ((carrot_point_y-currentY), (carrot_point_x-currentX)) *180/pi 
  if absTargetAngle < 0:
    absTargetAngle += 360
  turnError = absTargetAngle - currentHeading
  if (turnError > 180 or turnError < -180):
    turnError = -1 * sgn(turnError) * (360 - abs(turnError))

  linear_error = math.sqrt((targetX-currentX)**2 + (targetY-currentY)**2)
  angular_error = turnError

  linear_speed = linear_error * Kp_lin
  angular_speed = angular_error * Kp_turn

  return linear_speed, angular_speed




fig = plt.figure()
trajectory_lines = plt.plot([], '--', color='black')
trajectory_line = trajectory_lines[0]
heading_lines = plt.plot([], '-', color='red')
heading_line = heading_lines[0]
poses = plt.plot([], 'o', color='orange', markersize=10)
pose = poses[0]
rect_lines_1 = plt.plot([], '-', color='orange')
rect_lines_2 = plt.plot([], '-', color='orange')
rect_lines_3 = plt.plot([], '-', color='orange')
rect_lines_4 = plt.plot([], '-', color='orange')
rect_line_1 = rect_lines_1[0]
rect_line_2 = rect_lines_2[0]
rect_line_3 = rect_lines_3[0]
rect_line_4 = rect_lines_4[0]

# Graph Setup
plt.plot([initX, targetX], [initY, targetY], 'x',color='red',markersize=10)
plt.axis("scaled")
plt.xlim (min([initX, targetX])-5, max([initX, targetX])+5)
plt.ylim (min([initY, targetY])-5, max([initY, targetY])+5)

xs = [currentPos[0]]
ys = [currentPos[1]]

f = 0

def draw_square (length, center, orientation):
  global rect_line_1, rect_line_2, rect_line_3, rect_line_4
  corner1 = [center[0] + length/np.sqrt(2)*np.cos((orientation + 45) *pi/180), center[1] + length/np.sqrt(2)*np.sin((orientation + 45) *pi/180)]
  corner2 = [center[0] + length/np.sqrt(2)*np.cos((orientation + 135) *pi/180), center[1] + length/np.sqrt(2)*np.sin((orientation + 135) *pi/180)]
  corner3 = [center[0] + length/np.sqrt(2)*np.cos((orientation - 135) *pi/180), center[1] + length/np.sqrt(2)*np.sin((orientation - 135) *pi/180)]
  corner4 = [center[0] + length/np.sqrt(2)*np.cos((orientation - 45) *pi/180), center[1] + length/np.sqrt(2)*np.sin((orientation - 45) *pi/180)]
  rect_line_1.set_data([corner1[0], corner2[0]], [corner1[1], corner2[1]])
  rect_line_2.set_data([corner2[0], corner3[0]], [corner2[1], corner3[1]])
  rect_line_3.set_data([corner3[0], corner4[0]], [corner3[1], corner4[1]])
  rect_line_4.set_data([corner4[0], corner1[0]], [corner4[1], corner1[1]])


def robot_animation (frame) :
  global currentPos, currentHeading, f

  linearVel, turnVel = new_boomerang(currentPos, currentHeading, targetPos, targetHeading, Kp_lin, Kp_turn)

  if f < 20:
    linearVel, turnVel = 0, 0


  maxLinVelfeet = 200 / 60 * pi*4 / 12
  maxTurnVelDeg = 200 / 60 * pi*4 / 9 *180/pi


  leftSideVel = linearVel - turnVel
  rightSideVel = linearVel + turnVel
  stepDis = (leftSideVel + rightSideVel)/100 * maxLinVelfeet * dt/1000


  currentPos[0] += stepDis * np.cos(currentHeading*pi/180)
  currentPos[1] += stepDis * np.sin(currentHeading*pi/180)
  currentHeading += (rightSideVel - leftSideVel)/2/100 * maxTurnVelDeg * dt/1000

  currentHeading = currentHeading%360
  if currentHeading < 0: currentHeading += 360

  xs.append(currentPos[0])
  ys.append(currentPos[1])

  draw_square(0.75, currentPos, currentHeading)
  heading_line.set_data ([currentPos[0], currentPos[0] + 0.75*np.cos(currentHeading/180*pi)], [currentPos[1], currentPos[1] + 0.75*np.sin(currentHeading/180*pi)])
  pose.set_data ((currentPos[0], currentPos[1]))
  trajectory_line.set_data (xs, ys)
  f += 1

anim = animation.FuncAnimation (fig, robot_animation, frames = numOfFrames, interval = dt)
plt.show()
