cmake_minimum_required(VERSION 2.8.3)

project(manipulator)

find_package(catkin REQUIRED cmake_modules COMPONENTS
		controller_manager
		joint_state_controller
		robot_state_publisher
		joint_limits_interface
		roscpp
		rospy
		sensor_msgs
		geometry_msgs
		message_generation
		std_msgs
		tf)

find_package(roslaunch)

find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)

add_message_files(
  FILES
  Trajectory.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
		CATKIN_DEPENDS
			roscpp
			rospy
			sensor_msgs
			geometry_msgs
			std_msgs
			tf
			message_runtime
		DEPENDS
			EIGEN3)

include_directories(
        # include
        ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR}
)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

add_executable(RobotHWInterfaceNode src/RobotHWInterfaceNode.cpp
		src/RobotHWInterface.cpp src/RobotSerialComm.cpp src/CyclicBuffer.c)

add_executable(RobotTest src/RobotTest.cpp src/RobotHWInterface.cpp
	src/RobotSerialComm.cpp src/CyclicBuffer.c)

add_executable(RobotTestCircle src/RobotTestCircle.cpp src/RobotHWInterface.cpp
	src/RobotSerialComm.cpp src/CyclicBuffer.c)

add_executable(RobotTestLine src/RobotTestLine.cpp src/RobotHWInterface.cpp
	src/RobotSerialComm.cpp src/CyclicBuffer.c)

target_link_libraries(RobotHWInterfaceNode ${catkin_LIBRARIES} )
target_link_libraries(RobotTest ${catkin_LIBRARIES} )
target_link_libraries(RobotTestCircle ${catkin_LIBRARIES} )
target_link_libraries(RobotTestLine ${catkin_LIBRARIES} )
