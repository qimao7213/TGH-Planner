# TGH-Planner
Topology-Guided Hierarchical Planner for Nonholonomic Robots in Unknown and Complex Environments (Submitted to IEEE T-Mech). 

PDF Link: [TechRxiv](https://www.techrxiv.org/users/897374/articles/1274757-tgh-planner-topology-guided-hierarchical-planner-for-nonholonomic-robots-in-unknown-and-complex-environments?commit=1f84764f4edc345d91a809fd2cc0ea9d6597c29c).

Video：[bilibili](https://www.bilibili.com/video/BV1pK98YfEuY/?spm_id_from=333.1387.upload.video_card.click&vd_source=b0464106123611f8d997cd304ba81efe)

<p align="center">
    <img src="files/realworld_experiment.png" alt="Realworld Experiment" width="50%">
</p>

**Contributions:**
- Propose a more effiecient UVD (Uniform Visibility Deformation) to accelerate HEC (Homotopy Equivalence Check).
- Design a topological path set to store historical paths, which not only provides consistent and stable guiding trajectories but also significantly reduces re-planning time
- Consider nonholonomic dynamics and unkown obstacle risks in local trajectory optimization to ensure safety.

Please kindly star :star: this project if it helps you. We take great efforts to develope and maintain it :grin::grin:.

TGH-Planner is based on the the excellent work of [FAST-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner).

## 1. Build and Start
**Dependencies:** Ubuntn20.04, ROS1, OpenCV, xterm, CUDA (Optional, see FAST-Planner)
- Step1:
```
  cd ${YOUR_WORKSPACE_PATH}/src
  git clone https://github.com/qimao7213/TGH-Planner.git
```
- Step2:
Unzip the **3rd.zip** to a different path of your choice, and run the following codes for each folder:
```
  mkdir build && cd build && cmake ..
  make
  sudo make install
```
- Step3:
Change the *nlopt path* in "*fast_planner/bspline_opt/CMakeLists.txt Line 19*" to yours. 
Change the *FilePath* in "*fast_planner/plan_manage/kino_replan.launch Line 5*" to yours, if you want to save trajectory information and odometry information.

- Step4:
```
  catkin_make
```
If you meet: 
```
Could not find a package configuration file provided by "bspline" with any of the following names...
```
run
```
catkin_make -DCATKIN_WHITELIST_PACKAGES=bspline
```
and then run
```
catkin_make -DCATKIN_WHITELIST_PACKAGES=
```
- Step5:
Open three terminals and run the following commands respectively:
```
  roscore
```
```
  roslaunch plan_manage rviz.launch
```
```
  roslaunch plan_manage kino_replan.launch
```
Then you can select a goal in Rviz and the robot will move.

### **CUDA**

If you do not use CUDA, you can change the mode at "*fast_planner/uav_simulator/local_sensing/CMakeLists.txt Line 6*".

However, the mode without CUDA can only generate low-quality global map. We recommend using **CUDA** or using **Gazebo**.

### **Gazebo**

If you want to use the Gazebo world, instead of a randomly generated map, you can set the *use_gazebo* parameter to **true** in the "*fast_planner/plan_manage/launch/kino_replan.launch Line 7*" , and then set the *.world file path* in "*uav_simulator/gazebo_simulator/launch/gazebo_sim.launch Line 14*" to your .world path. If you want to open the Gazebo GUI, comment out "*Line 20*".

We are not using the physics simulation in Gazebo. Instead, we set the car pose calculated by *car_simulator* node to the robot in Gazebo using *set_robot_pose* service.

Change the parameters of *map_offset_x-y-yaw_* to fit the coordinates between Rviz and Gazebo.

### **TEB**
Run ``` sudo apt install ros-noetic-move-base ros-noetic-teb-local-planner ``` to install move_base and teb_planner.

You can switch planner algorithm to **TEB** by set the *use_teb* parameter to **true** in the *fast_planner/plan_manage/launch/kino_replan.launch Line 6*. Then, run ``` roslaunch plan_manage robot_carlike_in_stage.launch ``` in a new terminator and the robot will move guided by **TEB**.

The config files of **TEB** are at *fast_planner/plan_manage/teb_nav/*.

## 2. Some Utils
### Predifined Waypoints
Difine a set of waypoints in  "*fast_planner/plan_manage/config/waypts_2d.yaml*", and set "*fast_planner/plan_manage/launch/kino_replan.launch Line 88*" to 2. Then when you set any goal in Rviz, the robot will traverse the predefined waypoints.

### **Services**
Open ros service gui and you can see some services:
```
cd ${YOUR_WORKSPACE_PATH}/src
source devel/setup.bash
rosrun rqt_service_caller rqt_service_caller
```
- /car_simulator/set_init_pose

Set {x, y, yaw} pose to your robot. If the yaw > 360, the robot will be reset to the initial pose.
- /planning/reset_env

Reset the global map.
- /planning/traj_record

If you call a **True**, then the trajectory information will be recorded after the next planning is completed. The saved trajectory files are at "*test/traj_record*", **traj_cmd.txt** for the planned trajectory and **traj_real.txt** for the actual trajectory.

You can call a **False** to stop recording. If you set **Predifined Waypoints**, the recording will stop after traversing all waypoints.

In "*test/traj_record*", run
```
mkdir build && cd build
cmake .. && make
```
The executable files "calPathInfo" and "calTrajInfo" can be used to analyse trajectory information.

### **Record B-Spline Trajectory**
Set "*fast_planner/plan_manage/launch/kino_algorithm.xml Line 127*" to **true**, the latest optimized B-Spline Trajectories will be recored to "*test/data*". Use the "*.py*" functions to analyse the B-Spline Trajectories.

### **Draw Robot Trajectory on Map**
Use [collision_map_creator_plugin](https://github.com/tik0/collision_map_creator_plugin) to transform a Gazebo world to a "*.png*" 2D map.
Example code: 
```
./request_publisher "(-50,-10)(10,10)" 1 0.02 $(pwd)/map.png
```
In "*test/traj_record/src/drawTrajMap.cpp*", the *world_image_path* is the generated map.png above. 

The *origin_x* and *origin_y* are calculated as: 
*origin_xy* = *lowerleft.xy* - *map_offset_xy_*, where *lowerleft.xy* is (-50, -10) above, and *map_offset_xy_* is the parameters at "*uav_simulator/gazebo_simulator/launch/gazebo_sim.launch*".

## 3. Deployment on A Real Robot
We need **depth image**, **camera pose** and ?????
I will update this section later...

## 4. 
I will continue to improve this repository and enhance the code in the future.
