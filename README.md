# Waste Sorting - Robot Motion Planing Project
TODO - add a video here

* Waste sorting with multiple arms simulation using motion planing algorithms and multiprocessing
* Our package features a non-interruptive solution to motion planning by handling collision checks and motion planning computations in a sandbox simulation that runs in a different process in the background
* The result is a fluid, real-time simulation and a stable environment
  * Other motion planning projects compute motion planning inside the simulated environment, which affects (and ruins) the environment itself
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
### Featured Components and Project Structure
#### UR5 robot arm and Robotiq 2F-85 gripper
`multiarm_planner/ur5.py` `assets/ur5/` `assets/gripper/`
* Opening and closing of gripper
* State machine designed for waste sorting
 
Also contains features found in [Multiarm Motion planner](https://github.com/galmw/centralized-multiarm-drrt) (some with minor adjustments):
* Setup of arm and gripper - loading URDF models and setting gripper constraints
* Collision checking
* Inverse Kinematics
* Forward Kinematics
* Arm movement

#### Motion planning for multiple arms using BiRRT
`multiarm_planner/`
* Taken directly from [Multiarm Motion planner](https://github.com/galmw/centralized-multiarm-drrt)
  * Minor adjustments that separate the algorithm from the simulation
* BiRRT algorithm implementation - `multiarm_planner/rrt/rrt_connect.py`
* Assuming an initialized `MultiarmEnvironment` (`multiarm_planner/multiarm_environment.py`) - usage of multi-arm BiRRT is through the `birrt` method in `MultiarmEnvironment`
* `mutliarm_planner/ur5_group.py` includes methods for multi-arm operations, such as multi-arm forward kinematics, multi-arm collision checks and more

#### Conveyor
`conveyor.py`
* The conveyor is a simple box model, that causes objects on top of it to move forward by removing friction and adding velocity
* Objects that should not be "conveyed", such as the UR5 arms or trash that is picked by an arm, should be added to the `dont_convey` list
  * If an object was previously conveyed, its friction should be returned to the original value when it is no longer conveyed - otherwise, the behavior is unknown
* Conveyor speed is adjustable using the `speed` argument and represents the velocity that is given to the objects

#### Trash
`models/` `trash.py` `trash_generator.py` `trash_types.py` `configs/trash_configs.py`
* Trash models can be found in the `models/` directory
* Trash objects can be summoned manually using the `TrashGenerator` class
  * Otherwise, summon components already preiodically summon trash and should be used for simulation
* Trash location can be mirrored to the other side of the conveyor using the `signed_value` function in `configs/trash_configs.py`

##### Adding New Trash
* Add the URDF model to `models/`
* Create a new config for the trash in `configs/trash_configs.py`. The config should include:
  * Path to the URDF model
  * Location for spawning the trash - relative to the conveyor position
  * Points that the trash should be picked up from - relative to the trash object
    * For example, a water bottle should be picked up from a point close to the lid if it's standing and picked from above rather than the center
  * Trash type - for deciding which bin the trash should be thrown to
  * If, for some reason, mirroring the trash to the other side causes unexpected behavior with gripping points, `mirrored_gripping_points` field can be added to ensure correct behavior on mirrored trash location
* Random summon components should automatically include your new trash in the simulation
  * Other summon components should manually be given your trash config to do so

#### Summon Components
`summon_component.py` `configs/trash_configs.py`




#### Environment and Main Event Loop
`environment.py` `real_environment.py` `background_environment.py`
* The `Environment` class is responsible for loading the simulation environment, that is, all used models including: ur5 arms, trash bins, conveyor and trash generator
* As both the actual simulation and sandbox simulations are identical in their design, both `RealEnvironment` and `BackgroundEnvironment` classes inherit the parent `Environment` class and expand upon it
* `RealEnvironment` is the actual simulation and contains the main event loop including:
  * Running the trash summoner
  * Running the conveyor
  * Running the TaskManager state machine
  * Running each of the UR5 arms' state machine
  * Advancing the actual pybullet simulation
  * Keeping track of score
  * Removing redundant trash models from screen
* `BackgroundEnvironment` is the sandbox simulation used for various computations that require actions in the simulated world, such as moving a UR5 arm to the IK solution and verifying the solution is good enough, or running the motion planning algorithm to compute paths while checking for collisions. It also contains code for multiple sandbox simulations running in parallel using multiprocessing
* Background environments can be seen using the `Show background environments` option
* Simultaneously viewing both the real environment and the background environments is not supported

## Acknowledgments
[Multiarm Motion planner, Gal Wiernik](https://github.com/galmw/centralized-multiarm-drrt)
