cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)




# flags

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c++11 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native")

set(PGRFLYCAP_LIB_DIR "/usr/lib" CACHE PATH " ") # pointgrey location





##eigen
find_package(Eigen3)
include_directories(include ${EIGEN3_INCLUDE_DIR})
## ucoslam
FIND_PACKAGE(ucoslam 	REQUIRED )
LINK_LIBRARIES(${OpenCV_LIBS}  ${ucoslam_LIBS} )
## mynteyed



# When SDK build with OpenCV, we can add WITH_OPENCV macro to enable some
# features depending on OpenCV, such as ToMat().


## OpenCV

# Set where to find OpenCV
#set(OpenCV_DIR "/usr/share/OpenCV")

# When SDK build with OpenCV, we must find the same version here.
find_package(OpenCV REQUIRED)
message(STATUS "Found OpenCV: ${OpenCV_VERSION}")

# targets

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)



include_directories(include ${catkin_INCLUDE_DIRS})  

LIST(APPEND LINK_LIBS
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
  ##${SENSE_LIB_DIR}/librealsense.so
  ${PGRFLYCAP_LIB_DIR}/libflycapture.so
)
  
#add_executable(talker src/talker.cpp)
add_executable(talker src/pg_map_exposure.cpp include/pointgreyStream.h )
target_link_libraries(talker ${catkin_LIBRARIES} ${LINK_LIBS} )  
message(STATUS"${catkin_LIBRARIES}")
add_dependencies(talker beginner_tutorials_generate_messages_cpp)


add_executable(record src/record.cpp include/pointgreyStream.h)
target_link_libraries(record ${catkin_LIBRARIES} ${LINK_LIBS} )
add_dependencies(record beginner_tutorials_generate_messages_cpp)

add_executable(img2video src/img2video.cpp )



