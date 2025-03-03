论文：TGH-Planner: Topology-Guided Hierarchical Planner for Nonholonomic Robots in Unknown and Complex Environments （投稿IEEE T-Mech）

视频：[bilibili](https://www.bilibili.com/video/BV1pK98YfEuY/?spm_id_from=333.1387.upload.video_card.click&vd_source=b0464106123611f8d997cd304ba81efe)

论文链接后续更新

这个代码是根据FAST-Planner改的。仓库的内容后续再慢慢弄。

主要改进的地方：
（1）提升了UVD的效率；
（2）构建了一个拓扑路径容器，可以在全局引导的时候利用历史规划信息；
（3）优化时考虑了非全向机器人

依赖：Ubuntn20.04,ROS1,OpenCV,xterm,CUDA(没有也可以，具体参考FAST-Planner)

Step1: 克隆这个库，然后把里面打包好的3rd.zip依赖文件在其他地方解压，cmake .., make , sudo make install；

Step2: 把TGH-Planner/fast_planner/bspline_opt/CMakeLists.txt里面19行的nlopt路径改为自己的；

Step3：catkin_make编译；

Step4: 打开两个终端，分别运行： roslaunch plan_manage rviz.launch 和 roslaunch plan_mange kino_replan.launch

我自己测试过，应该是没有问题的。

很多地方后续再完善吧。


