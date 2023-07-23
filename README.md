# Maze exploring with mindstorms EV3

This is a Python script for a LEGO Mindstorms EV3 robot to explore a 3-floor maze. The robot actively updates its map based on sonic and color sensor observations. 

## Requirements
- LEGO Mindstorms EV3 set
- Python and Pybricks library installed on your EV3 brick

## Usage
1. Install Python and the Pybricks library on your LEGO Mindstorms EV3 brick.
2. Connect the motors and sensors to the appropriate ports on your EV3 brick according to the code configuration.
3. Upload the `main.py` script to your EV3 brick.
4. Run the script.

## Configuration
You can customize the robot's driving parameters, sonic and maze settings, and color recognition ranges by modifying the `Config` class in the script.

## Functionality
The script provides the following functionality:
- The robot can rotate its head in different directions to look for obstacles using the ultrasonic sensor.
- The robot uses a custom class `RGBClassifier` to recognize colors based on RGB values.
- The `Modules` class contains functions for the robot's observation, such as looking in front, right, left, and under the color sensor to identify cells and obstacles.
- The `GyroPIDController` class implements a PID controller using the gyro sensor to follow a desired yaw angle.
- The `CustomDriveBase` class extends the built-in drivebase and contains methods to update the robot's position, orientation, and maze map.
- The robot performs various actions based on the color of the cell it is on, such as saying the color aloud.

## Exploration Process
The `explore` method in the `CustomDriveBase` class is the main function for the maze exploration process. The robot continuously explores the maze, updating the map as it moves, and taking actions based on the cell color. The robot can handle hills leading to different floors and obstacles blocking its path.

## Note
This script is a basic implementation for a simple maze exploration task. There is room for improvement and optimization based on specific maze structures and requirements. 
