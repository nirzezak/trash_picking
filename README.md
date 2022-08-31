# Waste Sorting - Robot Motion Planing Project
TODO - add a video here

* Waste sorting with multiple arms simulation using motion planing algorithms and multiprocessing
* Project for the workshop in algorithms for robot motion planning in Tel Aviv University 	

## Requirements
* Use Python 3.9+ (former versions haven't been tested)
* The required Python libraries are defined in: `requirements.txt`

## User Instructions
### GUI
TODO

### CLI
* View usage instructions by running: `main.py --help`
* Usage: `main.py [-h] [--back] [--debug] [--arms ARMS] [--bins BINS]`
* Optional arguments:
  * -h, --help: show the help message and exit
  * --back: display the background environment in the GUI instead
  * --debug: print debug messages
  * --arms ARMS: JSON file that contain arms positions
  * --bins BINS: JSON file that contain bins positions
* The default configuration files are: `configs/arms_locations.json` and `configs/trash_bins_locations.json`. Use them as examples for configuration files
* Examples:
  * `python main.py` - runs the waste sorting simulation with the default configuration files (4 arms) 
  * `python main.py --arms "arms_conf.json" --bins "bins_conf.json"` - runs the waste sorting simulation with `arms_conf.json`, `bins_conf.json` configuration files
  * `python main.py --back` - run the waste sorting program but instead of presenting the sorting simulation, present the background environments in the GUI
 
## Package Details
### Components
#### UR5 robot arm and Robotiq 2F-85 gripper
* Opening and closing of gripper
* State machine designed for waste sorting
 
And also features found in Gal Wiernik's acknowledged repository (some with minor adjustments):
* Setup of arm and gripper - loading URDF models and setting gripper constraints
* Collision checking
* Inverse Kinematics
* Forward Kinematics
* Arm movement

All of which can be found in `multiarm_planner/ur5.py`

#### Motion planning for multiple arms using BiRRT
* Taken directly from [Multiarm Motion planner](https://github.com/galmw/centralized-multiarm-drrt)
  * Minor adjustments that separate the algorithm from the simulation
* All relevant code is in `multiarm_planner`
* BiRRT algorithm implementation - `multiarm_planner/rrt/rrt_connect.py`
* Assuming an initialized `MultiarmEnvironment` (`multiarm_planner/multiarm_environment.py`) - usage of multi-arm BiRRT is through the `birrt` method in `MultiarmEnvironment`
* `mutliarm_planner/ur5_group.py` includes methods for multi-arm operations, such as multi-arm forward kinematics, multi-arm collision checks and more

## Acknowledgments
[Multiarm Motion planner, Gal Wiernik](https://github.com/galmw/centralized-multiarm-drrt)
