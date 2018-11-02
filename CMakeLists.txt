cmake_minimum_required(VERSION 2.4.6)
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

project(but_velodyne_lib)
set (BUT_VELODYNE_VERSION_MAJOR 0)
set (BUT_VELODYNE_VERSION_MINOR 1)
set (BUT_VELODYNE_VERSION_PATCH 0)
set (BUT_VELODYNE_VERSION ${BUT_VELODYNE_VERSION_MAJOR}.${BUT_VELODYNE_VERSION_MINOR}.${BUT_VELODYNE_VERSION_PATCH})
set (BUT_VELODYNE_SOVERSION ${BUT_VELODYNE_VERSION_MAJOR}.${BUT_VELODYNE_VERSION_MINOR})
message(STATUS "Compiling But-Velodyne Version ${BUT_VELODYNE_VERSION}, soversion=${BUT_VELODYNE_SOVERSION}")

set(LIB_SUFFIX CACHE STRING "suffix for the library directory need for x86-64 systems that use lib64 ")
set(BUT_VELODYNE_INCLUDE_INSTALL_DIR "but_velodyne-${BUT_VELODYNE_VERSION_MAJOR}.${BUT_VELODYNE_VERSION_MINOR}" CACHE STRING "Directory name for header files")
set(BUT_VELODYNE_CMAKE_INSTALL_DIR "but_velodyne-${BUT_VELODYNE_VERSION_MAJOR}.${BUT_VELODYNE_VERSION_MINOR}" CACHE STRING "Directory to install the cmake config files.")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
#set(CMAKE_BUILD_TYPE RELEASE)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

add_definitions(-DBOOST_PARAMETER_MAX_ARITY=7)

include_directories( ${PROJECT_SOURCE_DIR}/include)

find_package( OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS})

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Eigen3 REQUIRED)
include_directories(EIGEN3_INCLUDE_DIR)

FIND_PACKAGE(Boost 1.40 COMPONENTS program_options REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

########################### Library: #########################################

file(GLOB header_files ${CMAKE_CURRENT_SOURCE_DIR}/include/but_velodyne/*.h)
install(FILES ${header_files} DESTINATION include/${BUT_VELODYNE_INCLUDE_INSTALL_DIR}/but_velodyne COMPONENT ${COMPONENT_PREFIX}dev)
file(GLOB header_files ${CMAKE_CURRENT_SOURCE_DIR}/include/velodyne_pointcloud/*.h)
install(FILES ${header_files} DESTINATION include/${BUT_VELODYNE_INCLUDE_INSTALL_DIR}/velodyne_pointcloud COMPONENT ${COMPONENT_PREFIX}dev)

add_library(but_velodyne SHARED src/CollarLinesRegistration.cpp src/CollarLinesRegistrationPipeline.cpp
src/ImageLine.cpp src/KeypointsCorrespondenceProjector.cpp src/LineCloud.cpp src/MoveEstimation.cpp
src/PointCloudLine.cpp src/PolarGridOfClouds.cpp src/PoseGraphEdge.cpp src/Termination.cpp
src/VelodynePointCloud.cpp src/Visualizer2D.cpp src/Visualizer3D.cpp
src/VelodyneSpecification.cpp src/CollarLinesFilter.cpp src/GroundDetectionDataGenerator.cpp
src/EigenUtils.cpp src/GlobalOptimization.cpp src/common.cpp src/Clustering.cpp src/NormalsEstimation.cpp
src/SubseqRegistration.cpp src/KittiUtils.cpp src/RegistrationCrossValidation.cpp src/CollarLinesValidation.cpp)
target_include_directories(but_velodyne PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(but_velodyne ${PCL_LIBRARIES})
target_link_libraries(but_velodyne ${OpenCV_LIBS})
target_link_libraries(but_velodyne ${Boost_LIBRARIES})

########################### Odometry estimation: #############################

add_executable(collar-lines-odom src/apps/collar-lines-odom.cpp)
target_link_libraries(collar-lines-odom but_velodyne)

add_executable(translation-by-mean src/apps/translation-by-mean.cpp)
target_link_libraries(translation-by-mean but_velodyne)

add_executable(cls-reg-to-map src/apps/cls-reg-to-map.cpp)
target_link_libraries(cls-reg-to-map but_velodyne)

add_executable(slice-clouds-sequence src/apps/slice-clouds-sequence.cpp)
target_link_libraries(slice-clouds-sequence but_velodyne)

add_executable(cls-reg-subsequences src/apps/cls-reg-subsequences.cpp)
target_link_libraries(cls-reg-subsequences but_velodyne)

add_executable(cluster-subsequences src/apps/cluster-subsequences.cpp)
target_link_libraries(cluster-subsequences but_velodyne)

add_executable(find-broken-subsequences src/apps/find-broken-subsequences.cpp)
target_link_libraries(find-broken-subsequences but_velodyne)

add_executable(pick-subsequences src/apps/pick-subsequences.cpp)
target_link_libraries(pick-subsequences but_velodyne)

add_executable(gicp src/apps/gicp.cpp)
target_link_libraries(gicp but_velodyne)

add_executable(transform-pcd src/apps/transform-pcd.cpp)
target_link_libraries(transform-pcd but_velodyne)

add_executable(print-graph-from-cls-clusters src/apps/print-graph-from-cls-clusters.cpp)
target_link_libraries(print-graph-from-cls-clusters but_velodyne)

add_executable(cluster-velodyne-points src/apps/cluster-velodyne-points.cpp)
target_link_libraries(cluster-velodyne-points but_velodyne)

add_executable(print-graph-from-points-clusters src/apps/print-graph-from-points-clusters.cpp)
target_link_libraries(print-graph-from-points-clusters but_velodyne)

add_executable(print-graph-from-points-picked src/apps/print-graph-from-points-picked.cpp)
target_link_libraries(print-graph-from-points-picked but_velodyne)

add_executable(estimate-planar-noise src/apps/estimate-planar-noise.cpp)
target_link_libraries(estimate-planar-noise but_velodyne)

add_executable(overlaping-frames src/apps/overlaping-frames.cpp)
target_link_libraries(overlaping-frames but_velodyne)

add_executable(print-graph-from-ransac-planes src/apps/print-graph-from-ransac-planes.cpp)
target_link_libraries(print-graph-from-ransac-planes but_velodyne)

add_executable(normals-estimation src/apps/normals-estimation.cpp)
target_link_libraries(normals-estimation but_velodyne)

add_executable(refined-clustering src/apps/refined-clustering.cpp)
target_link_libraries(refined-clustering but_velodyne)

add_executable(reduce-plane-thickness src/apps/reduce-plane-thickness.cpp)
target_link_libraries(reduce-plane-thickness but_velodyne)

add_executable(reduce-plane-thickness-by-ray src/apps/reduce-plane-thickness-by-ray.cpp)
target_link_libraries(reduce-plane-thickness-by-ray but_velodyne)

add_executable(estimate-gmm-ransac-planes src/apps/estimate-gmm-ransac-planes)
target_link_libraries(estimate-gmm-ransac-planes but_velodyne)

add_executable(pcl_labels_viewer src/apps/pcl_labels_viewer)
target_link_libraries(pcl_labels_viewer but_velodyne)

add_executable(cls-registration-of-picked-frames src/apps/cls-registration-of-picked-frames)
target_link_libraries(cls-registration-of-picked-frames but_velodyne)

add_executable(pcl_subsample_normals src/apps/pcl_subsample_normals.cpp)
target_link_libraries(pcl_subsample_normals but_velodyne)

add_executable(sync-velodyne-clouds src/apps/sync-velodyne-clouds.cpp)
target_link_libraries(sync-velodyne-clouds but_velodyne)

add_executable(calibrate-imu-velodyne-by-normals src/apps/calibrate-imu-velodyne-by-normals.cpp)
target_link_libraries(calibrate-imu-velodyne-by-normals but_velodyne)

add_executable(get-ranges src/apps/get-ranges.cpp)
target_link_libraries(get-ranges but_velodyne)

########################## Visualization apps: ###############################

add_executable(show-kitti-poses src/apps/show-kitti-poses.cpp)
target_link_libraries(show-kitti-poses but_velodyne)

add_executable(show_correspondences src/apps/show_correspondences.cpp)
target_link_libraries(show_correspondences but_velodyne)

add_executable(kitti-viewer src/apps/kitti-viewer.cpp)
target_link_libraries(kitti-viewer but_velodyne)

add_executable(cloud-eraser src/apps/cloud-eraser.cpp)
target_link_libraries(cloud-eraser but_velodyne)

add_executable(build-3d-model src/apps/build-3d-model.cpp)
target_link_libraries(build-3d-model but_velodyne)

add_executable(threshold-intensity src/apps/threshold-intensity.cpp)
target_link_libraries(threshold-intensity but_velodyne)

add_executable(hor-align-and-defile src/apps/hor-align-and-defile.cpp)
target_link_libraries(hor-align-and-defile but_velodyne)

add_executable(hor-align-from-ground-normal src/apps/hor-align-from-ground-normal.cpp)
target_link_libraries(hor-align-from-ground-normal but_velodyne)

add_executable(normalize-intensity src/apps/normalize-intensity.cpp)
target_link_libraries(normalize-intensity but_velodyne)

add_executable(show-registration-in-seq src/apps/show-registration-in-seq.cpp)
target_link_libraries(show-registration-in-seq but_velodyne)

add_executable(visualize-global-graph src/apps/visualize-global-graph.cpp)
target_link_libraries(visualize-global-graph but_velodyne)

add_executable(visualize-part-clouds src/apps/visualize-part-clouds.cpp)
target_link_libraries(visualize-part-clouds but_velodyne)

add_executable(showroom-demo src/apps/showroom-demo.cpp)
target_link_libraries(showroom-demo but_velodyne)

add_executable(cloud-intensities-normalization src/apps/cloud-intensities-normalization.cpp)
target_link_libraries(cloud-intensities-normalization but_velodyne)

add_executable(pick-sources src/apps/pick-sources.cpp)
target_link_libraries(pick-sources but_velodyne)

add_executable(visualize-corrections src/apps/visualize-corrections.cpp)
target_link_libraries(visualize-corrections but_velodyne)

########################## Loops detection/closure: ##########################

add_executable(extract-vfh src/apps/extract-vfh.cpp)
target_link_libraries(extract-vfh but_velodyne)

add_executable(find-loops-vfh src/apps/find-loops-vfh.cpp)
target_link_libraries(find-loops-vfh but_velodyne)

add_executable(verify-loops src/apps/verify-loops.cpp)
target_link_libraries(verify-loops but_velodyne)

add_executable(register-loops src/apps/register-loops.cpp)
target_link_libraries(register-loops but_velodyne)

add_executable(slampp-solution-to-poses src/apps/slampp-solution-to-poses.cpp)
target_link_libraries(slampp-solution-to-poses but_velodyne)

add_executable(gestalt-loops src/apps/gestalt-loops.cpp)
target_link_libraries(gestalt-loops but_velodyne)

add_executable(keypoints-matches src/apps/keypoints-matches.cpp)
target_link_libraries(keypoints-matches but_velodyne)

add_executable(correct-velodyne-distortion src/apps/correct-velodyne-distortion.cpp)
target_link_libraries(correct-velodyne-distortion but_velodyne)

add_executable(correct-velodyne-distortion-fine src/apps/correct-velodyne-distortion-fine.cpp)
target_link_libraries(correct-velodyne-distortion-fine but_velodyne)

add_executable(pose-graph-viewer src/apps/pose-graph-viewer.cpp)
target_link_libraries(pose-graph-viewer but_velodyne)

add_executable(cloud-eraser-from-seq src/apps/cloud-eraser-from-seq.cpp)
target_link_libraries(cloud-eraser-from-seq but_velodyne)

########################## RANSAC registration (visual features): ############

add_executable(manual-registration src/apps/manual-registration.cpp)
target_link_libraries(manual-registration but_velodyne)

########################## Moving object detection: ##########################

add_executable(move-detection src/apps/move-detection.cpp)
target_link_libraries(move-detection but_velodyne)

add_executable(ground-det-labeling src/apps/ground-det-labeling.cpp)
target_link_libraries(ground-det-labeling but_velodyne)

add_executable(ground-det-annotation src/apps/ground-det-annotation.cpp)
target_link_libraries(ground-det-annotation but_velodyne)

add_executable(ground-det-kitti-ann src/apps/ground-det-kitti-ann.cpp)
target_link_libraries(ground-det-kitti-ann but_velodyne)

add_executable(ground-by-median src/apps/ground-by-median.cpp)
target_link_libraries(ground-by-median but_velodyne)

add_executable(to-kitti src/mit-dgc-data/c/to-kitti.cpp
    src/mit-dgc-data/c/eventlog.c src/mit-dgc-data/c/lcmtypes_gps_to_local_t.c
    src/mit-dgc-data/c/lcmtypes_pose_t.c src/mit-dgc-data/c/lcmtypes_image_t.c
    src/mit-dgc-data/c/lcmtypes_velodyne_t.c src/mit-dgc-data/c/lcmtypes_laser_t.c
    src/mit-dgc-data/c/config.c src/mit-dgc-data/c/rotations.c
    src/mit-dgc-data/c/math_util.c src/mit-dgc-data/c/velodyne.c
    src/mit-dgc-data/c/config_util.c src/mit-dgc-data/c/camtrans.c)
target_link_libraries(to-kitti but_velodyne)
target_include_directories(to-kitti PUBLIC ${PROJECT_SOURCE_DIR}/mit-dgc-data)

add_executable(ground-labels-viewer src/apps/ground-labels-viewer.cpp)
target_link_libraries(ground-labels-viewer but_velodyne)

########################## Installation: #####################################

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/but_velodyne-config.cmake.in" "${CMAKE_CURRENT_BINARY_DIR}/but_velodyne-config.cmake" IMMEDIATE @ONLY)
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/but_velodyne-config-version.cmake.in" "${CMAKE_CURRENT_BINARY_DIR}/but_velodyne-config-version.cmake" IMMEDIATE @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/but_velodyne-config.cmake" "${CMAKE_CURRENT_BINARY_DIR}/but_velodyne-config-version.cmake"
    DESTINATION "lib${LIB_SUFFIX}/cmake/${BUT_VELODYNE_CMAKE_INSTALL_DIR}" COMPONENT ${COMPONENT_PREFIX}dev)

install(
TARGETS but_velodyne collar-lines-odom show-kitti-poses show_correspondences extract-vfh
    find-loops-vfh verify-loops register-loops slampp-solution-to-poses
    move-detection
RUNTIME DESTINATION bin/
LIBRARY DESTINATION lib/
INCLUDES DESTINATION include/
)
