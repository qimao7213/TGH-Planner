int32 order                         #阶数
int64 traj_id                       #该段轨迹编号
time start_time                     #该段轨迹开始时间

float64[] knots                     #位置控制点的节点向量，因为是非均匀的，所以必须要这个
geometry_msgs/Point[] pos_pts       #位置控制点（三维的）

float64[] yaw_pts                   #yaw角控制点
float64 yaw_dt                      #yaw角控制点的间隔，因为是均匀的，所以只需要这个就可以了

