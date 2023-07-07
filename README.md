# agora_robotics

Process to build and execute the code:

1. Create a workspace.
```
mkdir -p ~/ros2_ws/
```

2. Clone the repository inside `ros2_ws` folder.
``` 
git clone https://github.com/27Apoorva/agora_robotics.git
```

3. Run the build comamnd and source the workspace.
```
colcon build
source install/setup.bash
```

4. Execute the following commands one by one in each terminal.
```
rviz2
ros2 launch robot_localization ekf.launch.py 
ros2 run odom_tracker odom_tracker_node 
ros2 bag play bucur_square_drift5/bucur_square_drift5_0.db3 
```
5. Once robot has moved enough and you want to see the error plot, you can call the ros service.
```
ros2 service call /plot_and_save std_srvs/SetBool "{data: true}"

```

To run amcl:
```
ros2 launch nav2_bringup localization_launch.py use_sim_time:=False autostart:=True map:=agora_map.yaml params_file:=navigation2/nav2_bringup/params/nav2_params.yaml

```
AMCL:
1. Created a `.pgm` map from the bag using `nav2_map_server`. This is the 2D Occupancy map supplied to AMCL. 
2. Build `nav2_amcl` using `colcon build --packages-select nav2_amcl --allow-overriding nav2_amcl`
3. Use the map file name stored in `src `folder. Make sure before running the comamnd, you are in src folder path. The following command will run AMCL on the bag as well as robot.
```
ros2 launch nav2_bringup localization_launch.py use_sim_time:=False autostart:=True map:=agora_map.yaml params_file:=navigation2/nav2_bringup/params/nav2_params.yaml
```
4. Once launched, open `RViz`and use `2D Position` tool to give the initial pose of the robot to AMCL before robot starts moving.
5. You can echo the topic `amcl_pose` to see the pose estimate of AMCL.
6. For the bag, the tf already had map frame that is why you might see some jumping in tf on bag.
7. For the robot, this map to odom will come from AMCL. 
8. To get the robot’s pose, you can listen for tf between map and base_link and even publish it as a new topic at your desired frequency and supply it to the rest of the system.
