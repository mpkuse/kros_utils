cmake_minimum_required(VERSION 2.8.3)
project(kros_utils)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  std_msgs
  message_generation
  rosbag
  pcl_conversions
  pcl_ros
)
find_package(Boost REQUIRED COMPONENTS system)


find_package(OpenCV 3 REQUIRED)
find_package(PCL 1.2 REQUIRED)


find_package(Eigen3 REQUIRED)
include_directories(
  ${EIGEN3_INCLUDE_DIRS}
)

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}

)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


#FILE( GLOB KuseUtilsSources
    #src/utils/PoseManipUtils.cpp
    #src/utils/RosMarkerUtils.cpp
    #src/utils/MeshObject.cpp
    #src/utils/RawFileIO.cpp
    #src/utils/MiscUtils.cpp
    #src/utils/CameraGeometry.cpp
    #src/utils/PointFeatureMatching.cpp
    #src/utils/GMSMatcher/gms_matcher.cpp
#)

#add_library( KuseUtils
#    SHARED
#    ${KuseUtilsSources}
#)

add_executable( image_saver_mono
    src/image_saver_mono.cpp )


target_link_libraries(image_saver_mono
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${Boost_LIBRARIES}
 )

 add_executable( image_saver_stereo
     src/image_saver_stereo.cpp )

 target_link_libraries(image_saver_stereo
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}  )



add_executable( quaternion_to_rotation_matrix
    src/quaternion_to_rotation_matrix.cpp
    )

target_link_libraries( quaternion_to_rotation_matrix
    #KuseUtils
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )


 add_executable( unsplit_lidar_ptcld src/unsplit_lidar_ptcld.cpp )
 target_link_libraries( unsplit_lidar_ptcld  ${catkin_LIBRARIES} ${PCL_LIBRARIES} )
