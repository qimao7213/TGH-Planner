cmake_minimum_required(VERSION 3.0.2)
project(common_srvs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

## Declare the service files to be built
add_service_files(
  FILES
  load_or_save_global_map.srv
  reset_env.srv
  set_init_pose.srv
  traj_record.srv
)

## Generate added messages and services
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

