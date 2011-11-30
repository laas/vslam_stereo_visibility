cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#FIXME: detect properly package.
INCLUDE_DIRECTORIES(
  /home/thomas/Downloads/rss12_stereo_visibility/
  /home/thomas/profiles/laas/install/stable/include/
  /home/thomas/profiles/laas/install/stable/include/TooN
  /home/thomas/Downloads/levmar-2.5
  )
LINK_DIRECTORIES(
  /home/thomas/Downloads/rss12_stereo_visibility/
  /home/thomas/profiles/laas/install/stable/lib
  /home/thomas/Downloads/levmar-2.5
  )
SET(SRCS
  /home/thomas/Downloads/rss12_stereo_visibility/CameraPose.cpp
  /home/thomas/Downloads/rss12_stereo_visibility/SceneSingle.cpp
  /home/thomas/Downloads/rss12_stereo_visibility/Feature.cpp
  /home/thomas/Downloads/rss12_stereo_visibility/StereoPoint.cpp
  /home/thomas/Downloads/rss12_stereo_visibility/StereoRig.cpp
  /home/thomas/Downloads/rss12_stereo_visibility/Pose.cpp
  /home/thomas/Downloads/rss12_stereo_visibility/Pose_VO.cpp
  /home/thomas/Downloads/rss12_stereo_visibility/Ransac_PnP.cpp
  /home/thomas/Downloads/rss12_stereo_visibility/ThreePt_PnP.cpp
  /home/thomas/Downloads/rss12_stereo_visibility/Ransac_VO.cpp
  /home/thomas/Downloads/rss12_stereo_visibility/ThreePt_VO.cpp
  /home/thomas/Downloads/rss12_stereo_visibility/Putative.cpp
  /home/thomas/Downloads/rss12_stereo_visibility/CDHarris.cpp
  /home/thomas/Downloads/rss12_stereo_visibility/CDFast.cpp
  /home/thomas/Downloads/rss12_stereo_visibility/fast_9.cpp
  /home/thomas/Downloads/rss12_stereo_visibility/fast_10.cpp
  /home/thomas/Downloads/rss12_stereo_visibility/fast_11.cpp
  /home/thomas/Downloads/rss12_stereo_visibility/fast_12.cpp
  /home/thomas/Downloads/rss12_stereo_visibility/nonmax.cpp
  /home/thomas/Downloads/rss12_stereo_visibility/VisionLocalization.cpp
  /home/thomas/Downloads/rss12_stereo_visibility/VisibilityClassifier.cpp
  /home/thomas/Downloads/rss12_stereo_visibility/fasthessian.cpp
  /home/thomas/Downloads/rss12_stereo_visibility/integral.cpp
  /home/thomas/Downloads/rss12_stereo_visibility/surf.cpp
  /home/thomas/Downloads/rss12_stereo_visibility/ipoint.cpp
  /home/thomas/Downloads/rss12_stereo_visibility/utils.cpp
  /home/thomas/Downloads/rss12_stereo_visibility/opencvviewer.cpp
  /home/thomas/Downloads/rss12_stereo_visibility/openglviewer.cpp
)
find_package(GLUT)
find_package(OpenGL)

find_package(OpenCV REQUIRED)

rosbuild_add_boost_directories()

#common commands for building c++ executables and libraries
rosbuild_add_executable(vslam src/nodes/vslam.cpp ${SRCS})
target_link_libraries(vslam
  ${OPENCV_LIBRARIES}
  ${OpenCV_LIBS}
  ${OPENGL_LIBRARIES}
  ${GLUT_LIBRARIES}
  lapack
  levmar
)

#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
