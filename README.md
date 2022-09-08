# Waste Sorting - Robot Motion Planing Project

https://user-images.githubusercontent.com/93075510/188473174-31167e69-07ba-43f0-a3de-72f23b3dbe5d.mp4

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
 
## Acknowledgments
[Multiarm Motion planner, Gal Wiernik](https://github.com/galmw/centralized-multiarm-drrt)
