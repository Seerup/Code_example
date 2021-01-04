cmake_minimum_required(VERSION 2.8.3)
project(continuous_slopes)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################
## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   exo_joint_conf.msg
   exo_output.msg
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES continuous_slopes 
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)



add_library(continuous_slopes  
	src/Exo_interface.cpp 
	src/fixedNeuralNet.cpp 
	src/LinearAlgebra.cpp 
        src/Evolutionary.cpp 
        src/NEAT.1.2.1/population.cpp src/NEAT.1.2.1/neat.cpp 
        src/NEAT.1.2.1/genome.cpp src/NEAT.1.2.1/organism.cpp
        src/NEAT.1.2.1/species.cpp src/NEAT.1.2.1/gene.cpp
        src/NEAT.1.2.1/nnode.cpp src/NEAT.1.2.1/network.cpp
        src/NEAT.1.2.1/trait.cpp src/NEAT.1.2.1/innovation.cpp
        src/NEAT.1.2.1/link.cpp)

add_executable(neatNetwork 
        src/NEATmain.cpp 
	src/Exo_interface.cpp 
        src/NEAT.1.2.1/population.cpp src/NEAT.1.2.1/neat.cpp 
        src/NEAT.1.2.1/genome.cpp src/NEAT.1.2.1/organism.cpp
        src/NEAT.1.2.1/species.cpp src/NEAT.1.2.1/gene.cpp
        src/NEAT.1.2.1/nnode.cpp src/NEAT.1.2.1/network.cpp
        src/NEAT.1.2.1/trait.cpp src/NEAT.1.2.1/innovation.cpp
        src/NEAT.1.2.1/link.cpp)

add_dependencies(neatNetwork continuous_slopes_generate_messages_cpp)
target_link_libraries(neatNetwork ${catkin_LIBRARIES})



#add_dependencies(neatNetwork continuous_slopes_generate_messages_cpp)
#target_link_libraries(neatNetwork ${catkin_LIBRARIES})

add_executable(fixedNetwork 
		src/fixedNeuralNet.cpp 
		src/LinearAlgebra.cpp 
		src/Exo_interface.cpp 
		src/Evolutionary.cpp
		src/CPG.cpp)
target_link_libraries(fixedNetwork ${catkin_LIBRARIES})
add_dependencies(fixedNetwork continuous_slopes_generate_messages_cpp)

add_dependencies(fixedNetwork continuous_slopes_generate_messages_cpp)
target_link_libraries(fixedNetwork ${catkin_LIBRARIES})

add_executable(networkTester 
		src/networkTester.cpp 
		src/LinearAlgebra.cpp
		src/Exo_interface.cpp 
		src/Evolutionary.cpp
		src/CPG.cpp)

target_link_libraries(networkTester ${catkin_LIBRARIES})
add_dependencies(networkTester continuous_slopes_generate_messages_cpp)
















