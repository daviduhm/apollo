cmake_minimum_required(VERSION 2.8.3)
project(simulator_image_converter)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  std_msgs
  sensor_msgs
)

catkin_package(
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS cv_bridge image_transport roscpp std_msgs
)

find_package(OpenCV)

include_directories(include ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/simulator_image_converter.cpp)
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
