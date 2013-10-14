Annealing
========
Usage: ./anneal file1.obj file2.obj 
The output file is written to output_file.obj

Issues
=========
One remaining issue is intersecting meshes. This problem is most likely caused by an error in the triangle-triangle smallest distance algorithm.

Possible Improvements
=======
After testing, it seems that gradually reducing the minimum gap between shapes could help stop the clumping (if slower annealing doesn't work)
It might also be a good idea to set a nonlinear function to determine how far an object should be translated/ rotated, so it moves a lot if it very far away from another object, but moves a smaller percentage of that distance if it is closer.

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

Main issue is clumping - these paramters need to be adjusted somehow so the objects dont end up sticking next to each other.