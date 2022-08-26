import numpy as np
from numpy import pi
import matplotlib.pyplot as plt
import math
import matplotlib.animation as animation
from IPython import display

# Constants
initX, initY = 0, 0
targetX, targetY = 5, 5
currentHeading = 135
targetHeading = 180
tolerance = 0.1
Kp_lin = 30
Kp_turn = 5

numOfFrames = 120
dt = 50   # ms, interval between frames

currentPos = [initX, initY]
targetPos = [targetX, targetY]

def find_min_angle (targetHeading, currentHeading):
  turnAngle = targetHeading - currentHeading
  if turnAngle > 180 or turnAngle < -180 :
    turnAngle = -1 * np.sign(turnAngle) * (360 - abs(turnAngle))
  return turnAngle


def move_to_pose_step (currentPos, currentHeading, targetPos, targetHeading, Kp_lin, Kp_turn, r = 1, turnMax = 100, linMax = 70):
  currentX, currentY = currentPos[0], currentPos[1]
  targetX, targetY = targetPos[0], targetPos[1]
  
  # the angle between the line connecting the robot's current position to the target point and the x axis
  absTargetAngle = math.atan2 ((targetY-currentY), (targetX-currentX)) *180/pi

  # keep absTargetAngle between 0 and 360
  if absTargetAngle < 0:
    absTargetAngle += 360


  D = math.sqrt((targetX-currentX)**2 + (targetY-currentY)**2)
  alpha = find_min_angle(absTargetAngle, targetHeading)
  errorTerm1 = find_min_angle(absTargetAngle, currentHeading)

  beta = math.atan(r/D) *180 / pi

  if alpha < 0:
    beta = -beta

  if abs(alpha) < abs(beta):
    turnError = errorTerm1 + alpha
  else:
    turnError = errorTerm1 + beta

  
  linearVel = Kp_lin * D
  turnVel = Kp_turn * turnError

  # when close enough to the target
  closeToTarget = False
  if D < tolerance:
    closeToTarget = True
  if closeToTarget:
    # equation 3.13 
    linearVel =  Kp_lin * D * np.sign(math.cos(turnError *pi/180))  # in pct
    # when close enough to the target, overwrite the turn error output from intermediate direction calculations to prevent oscillations
    turnError = find_min_angle(targetHeading, currentHeading)
    turnVel = Kp_turn * math.atan(math.tan(turnError *pi/180)) *180/pi

  # cap the velocities
  if np.abs(linearVel) > linMax:
    linearVel = np.sign(linearVel) * linMax
  if np.abs(turnVel) > turnMax:
    turnVel = np.sign(turnVel) * turnMax

  # prioritize turning
  #if linearVel > (100 - np.abs(turnVel)):
    #linearVel = 100 - np.abs(turnVel)
  # # don't prioritize turning
  if linearVel > 100:
    linearVel = 100

  return linearVel, turnVel



fig = plt.figure()
trajectory_lines = plt.plot([], '--', color='black')
trajectory_line = trajectory_lines[0]
heading_lines = plt.plot([], '-', color='red')
heading_line = heading_lines[0]
poses = plt.plot([], 'o', color='orange', markersize=10)
pose = poses[0]
# draw a rectangle
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

  linearVel, turnVel = move_to_pose_step (currentPos, currentHeading, targetPos, targetHeading, Kp_lin, Kp_turn)

  if f < 20:
    linearVel, turnVel = 0, 0


  maxLinVelfeet = 200 / 60 * pi*4 / 12
  maxTurnVelDeg = 200 / 60 * pi*4 / 9 *180/pi


  leftSideVel = linearVel - turnVel
  rightSideVel = linearVel + turnVel
  #if np.abs(leftSideVel) > 100 : np.sign(leftSideVel)*leftSideVel
  #if np.abs(rightSideVel) > 100 : np.sign(rightSideVel)*rightSideVel
  stepDis = (leftSideVel + rightSideVel)/100 * maxLinVelfeet * dt/1000
  if np.abs(leftSideVel) > 1 and np.abs(rightSideVel) > 1:
    print(f"Current X: {currentPos[0]}")
    print(f"Current Y: {currentPos[1]}")
    print(f"Left Side Drivetrain velocity: {np.abs(leftSideVel)}")
    print(f"Right Side Drivetrain velocity: {np.abs(rightSideVel)}")

  # print(f"Linear Velocity: {linearVel}")
  # print(f"Turn Velocity: {turnVel}")

  # update x and y
  currentPos[0] += stepDis * np.cos(currentHeading*pi/180)
  currentPos[1] += stepDis * np.sin(currentHeading*pi/180)

  # update heading
  currentHeading += (rightSideVel - leftSideVel)/2/100 * maxTurnVelDeg * dt/1000

  currentHeading = currentHeading%360
  if currentHeading < 0: currentHeading += 360

  # rest of the animation code
  xs.append(currentPos[0])
  ys.append(currentPos[1])

  # draw robot, heading line, pose, and trajectory
  draw_square(0.75, currentPos, currentHeading)
  heading_line.set_data ([currentPos[0], currentPos[0] + 0.75*np.cos(currentHeading/180*pi)], [currentPos[1], currentPos[1] + 0.75*np.sin(currentHeading/180*pi)])
  pose.set_data ((currentPos[0], currentPos[1]))
  trajectory_line.set_data (xs, ys)
  f += 1

anim = animation.FuncAnimation (fig, robot_animation, frames = numOfFrames, interval = dt)
plt.show()



  # model: 200rpm drive with 18" width
  #               rpm   /s  circ   feet
  # maxLinVelfeet = 200 / 60 * pi*4 / 12
  #               rpm   /s  center angle   deg
  # maxTurnVelDeg = 200 / 60 * pi*4 / 9 *180/pi