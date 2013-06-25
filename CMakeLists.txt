cmake_minimum_required(VERSION 2.8.3)
project(entres)
find_package(catkin REQUIRED COMPONENTS message_generation)

#uncomment if you have defined messages
add_message_files(
  FILES
  InferenceResult.msg
)
#uncomment if you have defined services
add_service_files(
  FILES
  Infer.srv
)

generate_messages(
  #TODO DEPENDENCIES
)

catkin_package(
  DEPENDS rospy
  CATKIN_DEPENDS message_runtime 
  INCLUDE_DIRS 
  LIBRARIES
)
