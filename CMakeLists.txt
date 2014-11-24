cmake_minimum_required(VERSION 2.8.3)
project(entres)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation)

add_message_files(
  FILES
  InferenceResult.msg
)

add_service_files(
  FILES
  Infer.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  DEPENDS rospy
  CATKIN_DEPENDS message_runtime 
)
