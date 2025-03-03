# TGH-Planner
Topology-Guided Hierarchical Planner for Nonholonomic Robots in Unknown and Complex Environments (Submitted to IEEE T-Mech). 

Link: Come Soon.

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
Change the *nlopt path* in **TGH-Planner/fast_planner/bspline_opt/CMakeLists.txt Line 19** to yours. 
Change the *FilePath* in **TGH-Planner/fast_planner/plan_manage/kino_replan.launch Line 5** to yours, if you want to save trajectory information and odometry information.

- Step4:
```
  catkin_make
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

## 2. 
I will continue to improve this repository and enhance the code in the future.

I will introduce how to deploy TGH-Planner onto a actual robot.
