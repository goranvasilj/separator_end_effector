cmake_minimum_required(VERSION 3.0.2)
project(separator_end_effector)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  tf
  message_generation
  serial
  )

add_service_files(FILES separator_service.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS
        serial

)

include_directories(
  ${catkin_INCLUDE_DIRS}
  
)

add_executable(separator_end_effector src/separator_end_effector.cpp)


target_link_libraries(separator_end_effector
  ${catkin_LIBRARIES}
)

