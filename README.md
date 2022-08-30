# Contents of this file
 * Introduction
 * Requirements
 * User instructions

# Introduction
* Waste sorting with multiple arms simulation using motion planing algorithms and multiprocessing
* Project for the workshop in algorithms for robot motion planning in Tel Aviv university 	

# Requirements
* Use Python 3.9+ (former versions haven't been tested)
* The reqired Python libraries are defined in: `reqirements.txt`

# User instructions
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
* The default configuration files: `configs/arms_locations.json` and `configs/trash_bins_locations.json`, use them as examples for configuration files
* Examples:
  * `python main.py` - runs the waste sorting simulation with the default configuration files (4 arms) 
  * `python main.py --arms "arms_conf.json" --bins "bins_conf.json"` - runs the waste sorting simulation with `arms_conf.json`, `bins_conf.json` configuration files
  * `python main.py --back` - run the waste sorting program but instead of presenting the sorting simulation, present in the GUI all the background environments

# Acknowledgments
[Multiarm Motion planner, Gal Wiernik](https://github.com/galmw/centralized-multiarm-drrt)
