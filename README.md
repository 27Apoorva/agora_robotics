# agora_robotics

Process to build and execute the code:

1. Create a workspace.
```
mkdir -p ~/ros2_ws/src/
```

2. Clone the repository inside `src` folder.
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

