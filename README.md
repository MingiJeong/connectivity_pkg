
# Connectivity ROS package

* This package achieves connection map building (steiner tree) by robots and goal tasks(waypoint positions) by five robots. The allocation is mainly based on variant of auction in PA-4. However, the algorithm does not allow one robot to be assigned to more than one location to ensure connectivity.  
* There are two launch files: one launch file (`roslaunch connectivity_pkg connectivity_environment.launch`) can run all the environments and TF connection nodes, while the other launch file (`roslaunch connectivity_pkg waypoint_send.launch random_generate:=BOOL`) has a role of publishing waypoints (including terminal nodes and connection nodes) based on steiner tree approximation algorithm.
* It is robust to achieve random waypoints sent by ROS topic `/waypoints`, if an user setup `random_generate` as True. 
* It also pops up Rviz node automatically and users can easily monitor the task performance. The terminal nodes are visualized in red, while the connection nodes in blue.


## Requirements
* Tested on ROS kinetic (not sure about Melodic but it should work)
* custom_turtlebot3_description package installed.
* `multipledispatch` library for function overloading in Python.
    ```
    pip install multipledispatch
    ```
* `networkx` library for building network graph and using steiner tree approximation
    ```
    pip install --upgrade networkx
    ```

## Build
* Clone the repository into workspace ,e.g.,`ros_workspace/src/`
* run the command `catkin_make`

## Run
* 1st terminal

    ```
    source ros_workspace/devel/setup.sh
    roslaunch connectivity_pkg connectivity_environment.launch
    ```

    * Once run, you will see the pop-up of Gazebo and Rviz. 
    * It will start connection to the robot and TF broadcastor. There will be warning messages while TF being connected. If it pops up more than once, it is abnormal as the warning message shows. It will show only once as it gets connected correctly.

* 2nd terminal
    ```
    roslaunch connectivity_pkg waypoint_send.launch random_generate:=BOOL
    ```
    * random_generate:=BOOL. However, I tested mainly between 3 and 7. I checked it is working with 20 points. See the report. 
    * It will trigger the required tasks by the assignment, e.g., waypoint topic, waypoint allocate intention, service, action.
    * You can see the waypoints visualized at Rviz. 
    * Once that numbered task is done, this node will automatically shut down. Please give at least __5 seconds__ to initalize the robots in the first terminal. Then, launch this node again with another point_total arugment.   

