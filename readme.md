# Mini Project 2 for the Robotics 2 course @ RPI
**Local and Global Algorithms for Path Planning**

## Introductory Notes
This was developed in ROS noetic. It (roughly) implements the bug 2 and the 8-way wavefront local motion planners, and the probability road map global planner. The rapidly expanding random tree global planner was also attempted, but was not working properly at time of code submission.

It simulates a robot in Gazebo, roughly approximated as circular and including rectangular elements primarily to identify orientation visually. These elements are essentially repeated from project 1. Many aspects remain buggy and the code is far from perfect.

To perform much of the math and image manipulation used in some of the planners and code, numpy and scikit (particularly, scikit-image) are also required.

## Launching the base simulation
In order to launch the base simulation, do:
```
roslaunch teleop base.launch
```

This initializes all of the intermediate scripts for simulating sensors on/peturbing motion of the robot, starts a gazebo server instance, and spawn the robot in a random location as specified in the respective launch file.

To see the robot in the gazebo environmeny, you can launch in another terminal:
```
gzclient
```

Similarly, to see the robot within its own frame, you can launch the rviz preview of the robot using:
```
roslaunch maru2_description maru2_preview.launch
```

Finally, if you want to launch with the intent of driving the robot rather than simply starting the base simulation to run the algorithms, start the `drive.launch` file instead of the `base.launch` file: 
```
roslaunch teleop drive.launch
```

## Starting a local planner
Once the simulation is started with the base launch file, the local planners can be used to drive the robot towards various goal parts, as long as they exist within the configuration space.

To make the bug 2 algorithm move towards a point, launch the `bug2.launch` script with target positions. It defaults to (8,8) without any arguments. The additional `eval` argument also launches a timing node to track time and path length. This will output to the `local_motion` node's folder in `devel/`.
```
roslaunch local_motion bug2.launch x:=8 y:=8 [eval:=true/false]
```

The same general parameters can be used to make the 8-way wavefront algorithm move towards a set point as well. One more argument exists to determine if the algorithm should pause to display the potential field generated instead of directly saving it. When `pause` is set to `false`, the generated field is automatically saved to the `generated_wavefront` folder in the `devel/` directory the `local_motion` node.
```
roslaunch local_motion wavefront.launch x:=8 y:=8 [eval:=true/false] [pause:=true/false]
```

Evaluated timings and paths will be stored in the `paths` folder in the `devel/` directory for the `local_motion` node.

## Starting a global planner
Similarly, a global planner can be used to generate the path, which will then call upon a local algorithm to reach each of the planned points along the trajectory.

To start the Probability Road Map (PRM) planner, launch with:
```
roslaunch global_planner prm_planner.launch x:=8 y:=8
```
This will save the planned trajectory into the `plans` folder in the `devel/` directory for the `global_planner` node. And when the plan is executed upon, the overall path is evaluated with the `local_motion` package, and it stores data collected to the same place previously mentioned for `local_motion`.

The same style launch will work to launch the Rapdily expanding Random Tree (RRT) planner, though it may not succeed as the tree graphs are malformed.
```
roslaunch global_planner rrt_planner.launch x:=8 y:=8
```

## Auxilary/Utility
The robot mapper, which was used to generate the free space and configuration space maps of the room, can be started using:
```
roslaunch mapper octomap_mapping.launch
```

The map can then be exported from this after driving around using the map_saver on the occupancy grid messages. This saved occupancy grid map can then be moved to the appropriate location in the `maru2_map` source folder, from where the `clean_map.py` script can clean up the map into free space and generate the configuration space from there.

