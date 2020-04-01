# Implement the second move model for the Lego robot.
# The difference to the first implementation is:
# - added a scanner displacement
# - added a different start pose (measured in the real world)
# - result is now output to a file, as "F" ("filtered") records.
#
# 02_b_filter_motor_file
# Claus Brenner, 09 NOV 2012
from math import sin, cos, pi
from lego_robot import *

# This function takes the old (x, y, heading) pose and the motor ticks
# (ticks_left, ticks_right) and returns the new (x, y, heading).
def filter_step(old_pose, motor_ticks, ticks_to_mm, robot_width,
                scanner_displacement):

    # Find out if there is a turn at all.
    if motor_ticks[0] == motor_ticks[1]:
        # No turn. Just drive straight.
        theta = old_pose[2]
        x = old_pose[0]+motor_ticks[0]*ticks_to_mm*cos(theta)
        y = old_pose[1]+motor_ticks[1]*ticks_to_mm*sin(theta) #position of liDAR
        
        # --->>> Use your previous implementation.
        # Think about if you need to modify your old code due to the
        # scanner displacement?
        
        return (x, y, theta)

    else:
        # Turn. Compute alpha, R, etc.
        r = motor_ticks[1]*ticks_to_mm #left wheel walk length
        l = motor_ticks[0]*ticks_to_mm#right wheel walk length
        w = robot_width
        d = scanner_displacement
        alpha = (r-l)/w #truning angel
        R = l/alpha  #turning radios
        cx = old_pose[0] -d*cos(old_pose[2]) -(R + w/2)*sin(old_pose[2])#fitered center of turning point x
        cy = old_pose[1]- d*sin(old_pose[2])-(R + w/2)*-cos(old_pose[2])#fitered center of turning point y
        x = cx + (R+w/2)*sin(old_pose[2]+alpha) + d*cos(old_pose[2]+alpha)#fitered new x positiong of robot 
        y = cy - (R + w/2)*cos(old_pose[2]+alpha) + d*sin(old_pose[2]+alpha)#filtered new y positiong of robot 
        theta = (old_pose[2] +alpha)%(2*pi)
        # --->>> Modify your previous implementation.
        # First modify the the old pose to get the center (because the
        #   old pose is the LiDAR's pose, not the robot's center pose).
        # Second, execute your old code, which implements the motion model
        #   for the center of the robot.
        # Third, modify the result to get back the LiDAR pose from
        #   your computed center. This is the value you have to return.

        return (x, y, theta)

if __name__ == '__main__':
    # Empirically derived distance between scanner and assumed
    # center of robot.
    scanner_displacement = 30.0

    # Empirically derived conversion from ticks to mm.
    ticks_to_mm = 0.349

    # Measured width of the robot (wheel gauge), in mm.
    robot_width = 150.0

    # Measured start position.
    pose = (1850.0, 1897.0, 213.0 / 180.0 * pi)

    # Read data.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")

    # Loop over all motor tick records generate filtered position list.
    filtered = []
    for ticks in logfile.motor_ticks:
        pose = filter_step(pose, ticks, ticks_to_mm, robot_width,
                           scanner_displacement)
        filtered.append(pose)

    # Write all filtered positions to file.
    f = open("poses_from_ticks.txt", "w")
    for pose in filtered:
        print >> f, "F %f %f %f" % pose
    f.close()
