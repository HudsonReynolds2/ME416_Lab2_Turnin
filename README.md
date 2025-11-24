# ME416_Lab2_Turnin

The main script is the autonomous_navigation.m.

We did Kalman filter sidequest, spent lots of time on other sidequests and had working code that we did not get the chance to integrate with our working code.
A simple upgrade is to add some logic to backup and recompute path. We did not do this upgrade because we ran out of time.

Steps:
First, install matlab and regret it.
Then, power on the limo, ssh into it, run the limo remote control on it.

Place the robot facing +X in the room, at a place which will become the maze frame origin.

Then, run the calibrate_limo.m script with the correct robot mqtt topic and IP address.

Then, with the robot almost exactly where it was calibrated, run the autonomous_navigation.m script with the floor clear.

Watch the robot explore! But have your finger on the cancel button in case you need to stop it.

Simple changes to the calibration script if you prefer a different orientation, and you can easily validate your transforms by running test_transform_directions.m

To use a different maze, change the config file. 

visualize_all_mazes_all_paths.m was used to compare A*, DFS, and BFS outputs. While changes can be made to the config to get a good path for each maze with each algorithm, we were unable to unify these into a visualizer that showed good stuff all at once. We did not have time to fix this. but we did a lot of BFS and DFS in this lab and can show more of our stuff if needed.

Good luck exploring!
