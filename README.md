Annealing
========
Usage: ./anneal file1.obj file2.obj 
The output file is written to output_file.obj

Issues
=========
Crashes on loading some meshes.
Annealing speed may need to be slowed

Defines
========

These are defined in main.cpp. Perhaps they should be moved out into a json config file.

define GAP 0.4 - minimum spacing in mm between two objects in the annealing process
define PADDING 50.0f - minimim starting distance from an object to the edge of the box
define PERCENT_TRANSLATION 0.45f - how much should we translate one object, as a percentage of max_distance
define PERCENT_ROTATION 0.45f - how much should we rotate one object, as a percentage of max_distance
define ITERATIONS 1000 - iterations for the annealing process
define SPACING 20 - starting spacing between objects
define CUBE_SHRINKAGE_RATE 0.01 - how much smaller should the cube become

