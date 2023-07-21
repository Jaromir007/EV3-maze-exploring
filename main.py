#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# Settings for the robot and maze
class Config:
    WHEEL_DIAMETER = 56
    AXLE_TRACK = 115

    # Driving parameters
    DRIVE_SPEED = 100
    DRIVE_ACCELERATION = 200
    TURN_RATE = 100
    TURN_ACCELERATION = 400

    # sonic and maze
    SONIC_SPEED = 200
    HILL_DISTANCE = 320

# A magic class that recognizes colors from RGB
class RGBClassifier:

    # magic
    def euclidean_distance(self, color1, color2):
        return sum((c1 - c2) ** 2 for c1, c2 in zip(color1, color2)) ** 0.5

    def color_from_rgb(self, rgb):
        # color ranges
        # feel free to add your own colors, or extend those
        color_ranges = {
            "red" : (16, 2, 4),
            "green": (6, 15, 8),
            "yellow": (22, 18, 10),
            "white": (19, 26, 60),
            "grey": (14, 18, 38),
            "blue": (2, 7, 30)
        }

        closest_color = None
        closest_distance = float('inf')
        # find the closest color from the list 
        for color_name, color_range in color_ranges.items():
            distance = self.euclidean_distance(rgb, color_range)
            if distance < closest_distance:
                closest_color = color_name
                closest_distance = distance

        return closest_color

# Modules for robot's observation
class Modules():

    def __init__(self, medium_motor, sonic_sensor, color_sensor):
        # set class variables and objects
        self.sonic_rotation = "front"
        self.color_classifier = RGBClassifier()

    # rotate the head to a given position (from any turn direction)
    def rotate_head(self, direction):
        direction_angles = {
            "front": {"front": 0, "left": -90, "right": 90},
            "left": {"left": 0, "front": 90, "right": 180},
            "right": {"right": 0, "front": -90, "left": -180},
        }
        # TODO this data type is used to avoit if statements
        # TODO almost every if blocks in this code can be raplaced with such table
        # TODO and it will be more beautiful

        target_angle = direction_angles.get(self.sonic_rotation, {}).get(direction)
        medium_motor.run_angle(Config.SONIC_SPEED, target_angle, then=Stop.HOLD, wait=True)

    # functions for sonic_sensor (rotate the head do desired direction, then look and return cell type )
    def look_front(self):
        self.rotate_head("front")
        self.sonic_rotation = "front"

        # free space in the front
        if sonic_sensor.distance() > 50: 
            return "free"
        # there is a wall (detects only whole walls, not sides of the hills)
        else:
            return "obstacle"

    # same function for the right direction
    def look_right(self):  
        self.rotate_head("right")  
        self.sonic_rotation = "right"

        if sonic_sensor.distance() < 150:
            return "obstacle"
        else:
            return "free"
    
    # and left
    def look_left(self):  
        self.rotate_head("left")
        self.sonic_rotation = "left"

        if sonic_sensor.distance() < 150:
            return "obstacle"
        else:
            return "free"

    # color under the color sensor 
    def look_under(self):

        # firstly check the reflection, whether it is a hill or a normal cell
        if 10 <= color_sensor.reflection() <= 35:
            color = self.color_classifier.color_from_rgb(color_sensor.rgb())
            # straightly mark red as "obstacle" to make things simple
            if color == "red":
                color = "obstacle"
        # there is a hill leading to another floor
        elif color_sensor.reflection() > 35:
            color = "upHill"
        # hill again, but this time it leads to bottom floor
        elif 3 < color_sensor.reflection() < 10:
            color = "downHill"
        # if the reflection is less than 3, it is a hole, which can be also marked as "obstacle"
        else:
            color = "obstacle"
    
        return color

# PID followin desired yro anle (0)
class GyroPIDController:

    def __init__(self, gyro_sensor):
        self.last_error = 0
        self.integral = 0

        self.PROPORTIONAL = 5
        self.DERIVATIVE = 5
    # don't call directly, called from distance()
    # just a normal PI regulator (but i am still calling it PID)
    def _correct_position(self):
        error = gyro_sensor.angle()  
        p_fix = error * self.PROPORTIONAL

        derivative = self.last_error - error
        d_fix = derivative * self.DERIVATIVE

        self.last_error = error

        robot.drive(Config.DRIVE_SPEED, -p_fix - d_fix)  
    
    # drive using the gyro pid until desired distance
    def distance(self, distance: int | float):
        gyro_sensor.reset_angle(0)
        robot.reset()
        while robot.distance() < distance:
            self._correct_position()
            
        robot.stop()
        self.last_error = 0

# CustomDriveBase extends the built-in drivebase. 
# The robot has own map of the maze, 3D coordinates and orientation (north, east, south, west)
# The compass is relative to the 0 point of the map (maze)
class CustomDriveBase(DriveBase):

    def __init__(self, left_motor, right_motor, medium_motor, sonic_sensor, gyro_sensor, color_sensor, wheel_diameter, axle_track):
        super().__init__(left_motor, right_motor, wheel_diameter, axle_track)

        # coordinates and orientation
        self.floor = 0
        self.x = 4 
        self.y = 3
        self.orientation = "south"

        # new objects 
        self.maze = [[["unexplored" for _ in range(9)] for _ in range(6)] for _ in range(3)]
        self.pid = GyroPIDController(gyro_sensor)
        self.modules = Modules(medium_motor, sonic_sensor, color_sensor)

    # Updatee the robot's orientation by the turn direction
    def update_orientation(self, movement):
        if movement == "left":
            if self.orientation == "north":
                self.orientation = "west"
            elif self.orientation == "west":
                self.orientation = "south"
            elif self.orientation == "south":
                self.orientation = "east"
            elif self.orientation == "east":
                self.orientation = "north" 
        else:
            if self.orientation == "north":
                self.orientation = "east"
            elif self.orientation == "east":
                self.orientation = "south"
            elif self.orientation == "south":
                self.orientation = "west"
            elif self.orientation == "west":
                self.orientation = "north"

    # Update the robot's position on the grid map 
    def update_position(self, movement):
        if movement == "forward":
            if self.orientation == "north":
                self.y -= 1
            elif self.orientation == "east":
                self.x += 1
            elif self.orientation == "south":
                self.y += 1
            elif self.orientation == "west":
                self.x -= 1
        else:  
            if self.orientation == "north":
                self.y += 1
            elif self.orientation == "east":
                self.x -= 1
            elif self.orientation == "south":
                self.y -= 1
            elif self.orientation == "west":
                self.x += 1
    
    # Update the cell in front of the robot
    def update_maze_front(self):
        front_x = self.x
        front_y = self.y

        if self.orientation == "north": 
            front_y -= 1
        elif self.orientation == "east":
            front_x += 1
        elif self.orientation == "south":
            front_y += 1
        elif self.orientation == "west":
            front_x -= 1

        # check if the cell is in the map boundaries
        if 0 <= front_y < 6 and 0 <= front_x < 9:
            # if the cell is unexplored, look
            if self.maze[self.floor][front_y][front_x] == "unexplored":
                self.maze[self.floor][front_y][front_x] = self.modules.look_front()

    # Update cell on the right
    def update_maze_right(self):
        right_x = self.x 
        right_y = self.y

        if self.orientation == "north":
            right_x += 1
        elif self.orientation == "east":
            right_y += 1
        elif self.orientation == "south":
            right_x -= 1
        elif self.orientation == "west":
            right_y -= 1

        # check if the cell is in the map boundaries
        if 0 <= right_y < 6 and 0 <= right_x < 9:
            # if the cell is unexplored, look
            if self.maze[self.floor][right_y][right_x] == "unexplored":
                self.maze[self.floor][right_y][right_x] = self.modules.look_right()

    # Update cell on the left
    def update_maze_left(self):
        left_x = self.x
        left_y = self.y

        if self.orientation == "north":
            left_x -= 1
        elif self.orientation == "east":
            left_y -= 1
        elif self.orientation == "south":
            left_x += 1
        elif self.orientation == "west":
            left_y += 1

        # check if the cell is in the map boundaries
        if 0 <= left_y < 6 and 0 <= left_x < 9:
            # if the cell is unexplored, look
            if self.maze[self.floor][left_y][left_x] == "unexplored":
                self.maze[self.floor][left_y][left_x] = self.modules.look_left()

    # Update the front cell color
    def update_maze_under_front(self):
        front_x = self.x
        front_y = self.y

        if self.orientation == "north":
            front_y -= 1
        elif self.orientation == "east":
            front_x += 1
        elif self.orientation == "south":
            front_y += 1
        elif self.orientation == "west":
            front_x -= 1

        # check if the cell is in the map boundaries
        if 0 <= front_y < 6 and 0 <= front_x < 9:
            # check if the cell is not an obstacle
            if self.maze[self.floor][front_y][front_x] != "obstacle":
                # move a bit forward and check with the color sensor
                # then update the maze map
                robot.straight(100) 
                self.maze[self.floor][front_y][front_x] = self.modules.look_under()
                robot.straight(-100) 

    # Update the maze right under the color sensor, without moving
    def update_maze_under(self):
        # to avoid redundant operations
        if self.maze[self.floor][self.y][self.x] == "unexplored":
            self.maze[self.floor][self.y][self.x] = self.modules.look_under()

    # Update everything in the maze map, the robot will look only to "unexplored" spaces
    # So you can call that function just to make sure everything is marked, if so, it won't do anything
    def update_maze(self):
        self.update_maze_under()
        self.update_maze_right()
        self.update_maze_front()
        self.update_maze_under_front()
        self.update_maze_left()
    
    # turn for a given angle, then correct the error with gyro_sensor
    def gyro_turn(self, ang):
        initial_angle = gyro_sensor.angle()
        self.turn(ang)
        angle_diff = ang - (gyro_sensor.angle() - initial_angle)
        # it can even handle negative turns!
        while abs(angle_diff) > 1:
            turn_correction = angle_diff * 0.5
            self.turn(turn_correction)
            angle_diff = ang - (gyro_sensor.angle() - initial_angle)

    # No comment here
    def dance(self):

        self.straight(20)
        self.straight(-20)
        self.turn(20)
        self.turn(-20)
    
    # performs action according to the cell color
    def action(self, cell):
        if cell == "blue":
            ev3.speaker.say("blue")

        elif cell == "yellow":
            ev3.speaker.say("yellow")

        elif cell == "green":
            ev3.speaker.say("green")

    # Basically main function for the maze exploration process, called as robot.explore()
    # TODO there is still a lot of space for improvement
    def explore(self):

        self.update_maze_under()
        self.dance()

        while True:
            #  look around
            self.update_maze()

            # identify the surrounding cells - Front
            dy, dx = 0, 0
            if self.orientation == "north":
                dy = -1
            elif self.orientation == "east":
                dx = 1
            elif self.orientation == "south":
                dy = 1
            elif self.orientation == "west":
                dx = -1

            front_x, front_y = self.x + dx, self.y + dy

            # Right
            right_dx, right_dy = 0, 0
            if self.orientation == "north":
                right_dx = 1
            elif self.orientation == "east":
                right_dy = 1
            elif self.orientation == "south":
                right_dx = -1
            elif self.orientation == "west":
                right_dy = -1

            right_x, right_y = self.x + right_dx, self.y + right_dy

            #  Left
            left_dx, left_dy = 0, 0
            if self.orientation == "north":
                left_dx = -1
            elif self.orientation == "east":
                left_dy = -1
            elif self.orientation == "south":
                left_dx = 1
            elif self.orientation == "west":
                left_dy = 1

            left_x, left_y = self.x + left_dx, self.y + left_dy

            # if the cells are not in the boundaries, mark them as "obstacle"
            # doesnt update the maze!
            # this is just for one cycle of the while loop!
            if 0 <= front_y < 6 and 0 <= front_x < 9:
                front_cell = self.maze[self.floor][front_y][front_x]
            else:
                front_cell = "obstacle"

            if 0 <= left_y < 6 and 0 <= left_x < 9:
                left_cell = self.maze[self.floor][left_y][left_x]
            else:
                left_cell = "obstacle"

            if 0 <= right_y < 6 and 0 <= right_x < 9:
                right_cell = self.maze[self.floor][right_y][right_x]
            else:
                right_cell = "obstacle"

            # When the front cell is free, perform the action according to the type of the cell
            if front_cell != "obstacle":
                # If the cell is any known version of a hill 
                if front_cell == "downHill":
                    self.pid.distance(Config.HILL_DISTANCE) # TODO never tested and calibrated
                    self.update_position("forward")
                elif front_cell == "upHill":
                    self.pid.distance(Config.HILL_DISTANCE)
                    self.update_position("forward")
                # If the cell is anything else
                else:
                    self.pid.distance(270) 
                    self.update_position("forward")
                    self.action(front_cell)

            # When the front cell isn't free, check the right cell (and turn there if free)
            elif right_cell != "obstacle":
                self.gyro_turn(90)
                self.update_orientation("right")

            # When the right cell isn't free, check the left cell (and turn there if free)
            elif left_cell != "obstacle":
                self.gyro_turn(-90)
                self.update_orientation("left")
            # BTW these elifs determine the pimary turn direction of the robot

            # If there are no cells to move, just go back
            # TODO make better logic here, this is not working
            else:
                self.pid.distance(-270) 
                self.update_position("backward")

####################### Port settings and object initialisation ###################

ev3 = EV3Brick()

# motors
left_motor = Motor(Port.A, positive_direction=Direction.CLOCKWISE, gears=None)
right_motor = Motor(Port.B, positive_direction=Direction.CLOCKWISE, gears=None)
medium_motor = Motor(Port.C, positive_direction=Direction.CLOCKWISE, gears=[12, 36]) 

# sensors
color_sensor = ColorSensor(Port.S1)
sonic_sensor = UltrasonicSensor(Port.S2)
gyro_sensor = GyroSensor(Port.S3)

# new drivebase
robot = CustomDriveBase(left_motor, right_motor, medium_motor, sonic_sensor, gyro_sensor, color_sensor, Config.WHEEL_DIAMETER, Config.AXLE_TRACK)
robot.settings(Config.DRIVE_SPEED, Config.DRIVE_ACCELERATION, Config.TURN_RATE, Config.TURN_ACCELERATION)

###################################################################################

# explore the maze forever
robot.explore()
