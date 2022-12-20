# Hephaestus
 - Implemented behaviors for mapping, planning, and execution on the Webots Tiago robot
using Python. 
 - Components used include mapping using homogenous transformation, robot localization with odometry and
gps/compass, inverse kinematics for both arm and robot manipulation, camera vision, attempted object localization
using k-means clustering, obstacle avoidance, RRT with Path Smoothing, and behavior trees.

## Running this controller:
* Install all required packages: ikpy, pytrees, numpy, scipy, imgaeio, and pyploy
* Add two side cameras on the robot
* Multisense s21 Camera in front

## File Structure
The bulk of our code can be found in ./Hephaestus/controllers/grocery_shopper/behaviors. This folder contains a host of .py files which make up our controllers behavior tree, denotated with the suffix 'bt' in the file name. The behavior folder also contains a folder named models. Within models, one will find most of the other code. 
