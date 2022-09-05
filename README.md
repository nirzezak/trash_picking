# Waste Sorting - Robot Motion Planing Project

https://user-images.githubusercontent.com/93075510/188472783-c36c7705-d80d-4cff-bceb-fd36c576c630.mp4

* Waste sorting with multiple arms simulation using motion planing algorithms and multiprocessing
* Our package features a non-interruptive solution to motion planning by handling collision checks and motion planning computations in a sandbox simulation that runs in a different process in the background
* The result is a fluid, real-time simulation and a stable environment
  * Other motion planning projects compute motion planning inside the simulated environment, which affects (and ruins) the environment itself, while being visible (and visually unappealing) to the user
* Project for the workshop in algorithms for robot motion planning in Tel Aviv University 	

## Requirements
* Use Python 3.9+ (former versions haven't been tested)
* The required Python libraries are defined in: `environment.yml` (to be used with Anaconda)

## User Instructions
### GUI
The package has a GUI for easier control of the different configuration options.
It adds onto the configuration options defined by the CLI, and adds even more 
configuration options:
* Set the log level to `DEBUG` instead of `INFO`
* Show the background environments instead of the main simulation
* Choose the JSON file for the arms positions
* Choose the JSON file for the bins locations
* Choose the summon component to be used. Options available are: `AdvancedRandomSummonComponent`, `RandomSummonComponent` 
  and `DeterministicSummonComponent`, with either mustard, metal can, or paper box. 
  Note that there are other summon components, but they are mostly used for debugging.
* Choose the task manager to be used. Options available are: `AdvancedParallelTaskManager`,
  `ParallelTaskManager`, and `SimpleTaskManager`.

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
 
## Package Details and Programming Guidelines
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
  * Otherwise, summon components already periodically summon trash and should be used for simulation
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
* It is highly recommended to use [Blender](https://www.blender.org/) with the [Phobos](https://github.com/dfki-ric/phobos) add-on to calculate anything related to the trash model

#### Summon Components
`summon_component.py` `configs/trash_configs.py`
* Contains a few summoning components responsible for periodically spawning trash - used by `RealEnvironment` to simulate trash "arriving to the waste sorting facility", and creating tasks for said trash - simulating a camera used for object detection
* Summon components differ in trash objects, position, timing and amount
* Before adding a new summon component, please make sure to inherit the `SummonComponent` base class, and implement the abstract `step` method
* For more information on how to use and configure summon components check the documentation in `summon_component.py`

#### Tasks and Task Manager
`task_manager.py` `task.py`
* The `Task` object is the basic unit used by the different task managers. It contains
  info about the task such as the trash objects and arms related to the tasks, the starting tick
  of the task, etc.
* The task managers differ by their capabilities (from `SimpleTaskManager` to `AdvancedParallelTaskManager`
* The various task managers that can be used by the package can be found in `task_manager.py`. The
  most simple one, `SimpleTaskManager`, is the basis for task managers, and can be inspected in order to understand
  how to write a task manager
* However, it stops the simulation in order to do its path computations, which are expensive
* In order to avoid that, we developed parallel task managers, which use the `multiprocessing` module of python
  to create parallel background environments (their code can be found in `background_environment.py`). One should only use
  the parallel task managers, if they are not trying to debug something, since they simulate the real world more accurately
* Before adding a new task manager, please make sure to inherit the `TaskManagerComponent` base class (or a suitable 
  class, if it only extends some capabilities of other task managers), and implement 
  the abstract `add_trash` and `step` methods
* *Warning*: The `AdvancedTaskManager` is deprecated, and shouldn't be used
* For more information on how to use and configure task managers check the documentation in `task_manager.py`


#### Environment and Main Event Loop
`environment.py` `real_environment.py` `background_environment.py`
* The `Environment` class is responsible for loading the simulation environment, that is, all used models including: ur5 arms, trash bins, conveyor and trash generator
* As both the actual simulation and sandbox simulations are identical in their design, both `RealEnvironment` and `BackgroundEnvironment` classes inherit the parent `Environment` class and expand upon it
* The layout of the environment consists of:
  * Conveyor
  * UR5 arms on both sides of the conveyor - with the 2 closest arms on each side being a pair
  * Trash bins behind each arm - with one shared bin between each 2 arms in a pair
* See `User Instructions` above for instructions on how to change the amount and locations of arms and bins 
* `RealEnvironment` is the actual simulation and contains the main event loop including:
  * Running the trash summoner
  * Running the conveyor
  * Running the TaskManager state machine
  * Running each of the UR5 arms' state machine
  * Advancing the actual pybullet simulation
  * Keeping track of score
  * Removing redundant trash models from screen
* `BackgroundEnvironment` is the sandbox simulation used for various computations that require actions in the simulated world, such as verifying IK solution is good enough by moving the UR5 arm to the IK solution, or running the motion planning algorithm to compute paths while checking for collisions. It also contains code for multiple sandbox simulations running in parallel using multiprocessing
* `background_environment.py` is also the main module responsible for handling path computations: it is given the starting position and goals to reach (picking points of the trash and delivering points to bin), and implements various solutions to improve the speed and stability of finding appropriate paths
  * One can implement their own path computations to said goals if one wishes to - for more information see `background_environment.py`
* Background environments can be seen using the `Show background environments` option
* Simultaneously viewing both the real environment and the background environments is not supported

#### Information on anything not specified above can be found in the documentation inside the source code

## Acknowledgments
[Multiarm Motion planner, Gal Wiernik](https://github.com/galmw/centralized-multiarm-drrt)
