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
ADD_DEFINITIONS(-D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE)
INCLUDE_DIRECTORIES(
  src/rss12_stereo_visibility/
  /home/thomas/profiles/laas/install/stable/include/
  /home/thomas/profiles/laas/install/stable/include/TooN
  /home/thomas/Downloads/levmar-2.5
  )
LINK_DIRECTORIES(
  src/rss12_stereo_visibility/
  /home/thomas/profiles/laas/install/stable/lib
  /home/thomas/Downloads/levmar-2.5
  )
SET(SRCS
  src/rss12_stereo_visibility/CameraPose.cpp
  src/rss12_stereo_visibility/SceneSingle.cpp
  src/rss12_stereo_visibility/Feature.cpp
  src/rss12_stereo_visibility/StereoPoint.cpp
  src/rss12_stereo_visibility/StereoRig.cpp
  src/rss12_stereo_visibility/Pose.cpp
  src/rss12_stereo_visibility/Pose_VO.cpp
  src/rss12_stereo_visibility/Ransac_PnP.cpp
  src/rss12_stereo_visibility/ThreePt_PnP.cpp
  src/rss12_stereo_visibility/Ransac_VO.cpp
  src/rss12_stereo_visibility/ThreePt_VO.cpp
  src/rss12_stereo_visibility/Putative.cpp
  src/rss12_stereo_visibility/CDHarris.cpp
  src/rss12_stereo_visibility/CDFast.cpp
  src/rss12_stereo_visibility/fast_9.cpp
  src/rss12_stereo_visibility/fast_10.cpp
  src/rss12_stereo_visibility/fast_11.cpp
  src/rss12_stereo_visibility/fast_12.cpp
  src/rss12_stereo_visibility/nonmax.cpp
  src/rss12_stereo_visibility/VisionLocalization.cpp
  src/rss12_stereo_visibility/VisibilityClassifier.cpp
  src/rss12_stereo_visibility/fasthessian.cpp
  src/rss12_stereo_visibility/integral.cpp
  src/rss12_stereo_visibility/surf.cpp
  src/rss12_stereo_visibility/ipoint.cpp
  src/rss12_stereo_visibility/utils.cpp
  src/rss12_stereo_visibility/opencvviewer.cpp
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
