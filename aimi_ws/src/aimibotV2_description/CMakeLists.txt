cmake_minimum_required(VERSION 2.8.3)

project(aimibot)

find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)

install(PROGRAMS
  scripts/goal
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
