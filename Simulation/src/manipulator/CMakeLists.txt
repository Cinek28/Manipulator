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
		std_msgs
		tf)

find_package(roslaunch)

find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)

catkin_package(
		CATKIN_DEPENDS
			roscpp
			rospy
			sensor_msgs
			geometry_msgs
			std_msgs
			tf
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

add_executable(kdl_dkin src/kdl_dkin.cpp)
target_link_libraries(kdl_dkin ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} )
add_executable(RobotHWInterfaceNode src/RobotHWInterfaceNode.cpp src/RobotHWInterface.cpp)
target_link_libraries(RobotHWInterfaceNode ${catkin_LIBRARIES} )
