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

# Locate GTest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

########################### Library: #########################################

file(GLOB header_files ${CMAKE_CURRENT_SOURCE_DIR}/include/but_velodyne/*.h)
install(FILES ${header_files} DESTINATION include/but_velodyne COMPONENT ${COMPONENT_PREFIX}dev)
file(GLOB header_files ${CMAKE_CURRENT_SOURCE_DIR}/include/velodyne_pointcloud/*.h)
install(FILES ${header_files} DESTINATION include/velodyne_pointcloud COMPONENT ${COMPONENT_PREFIX}dev)

add_library(but_velodyne SHARED src/CollarLinesRegistration.cpp src/CollarLinesRegistrationPipeline.cpp
src/ImageLine.cpp src/KeypointsCorrespondenceProjector.cpp src/LineCloud.cpp src/MoveEstimation.cpp
src/PointCloudLine.cpp src/PolarGridOfClouds.cpp src/Termination.cpp
src/VelodynePointCloud.cpp src/Visualizer3D.cpp src/VelodyneSpecification.cpp src/CollarLinesFilter.cpp
src/EigenUtils.cpp src/common.cpp
src/SubseqRegistration.cpp src/KittiUtils.cpp src/RegistrationCrossValidation.cpp src/CollarLinesValidation.cpp
src/Overlap.cpp src/InterpolationSE3.cpp src/Calibration.cpp src/AdaptiveIntensitiesNormalization.cpp
src/DenseCloudRegistration.cpp src/VelodyneMultiFrameSequence.cpp src/PhaseFilter.cpp)
target_include_directories(but_velodyne PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(but_velodyne ${PCL_LIBRARIES})
target_link_libraries(but_velodyne ${OpenCV_LIBS})
target_link_libraries(but_velodyne ${Boost_LIBRARIES})

########################### Tests: #########################################

file(MAKE_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}/tests)

# Link runTests with what we want to test and the GTest and pthread library
add_executable(tests/RegistrationCrossValidation-test src/tests/RegistrationCrossValidation-test.cpp)
target_link_libraries(tests/RegistrationCrossValidation-test ${GTEST_LIBRARIES} pthread but_velodyne)

add_executable(tests/CollarLinesRegistration-test src/tests/CollarLinesRegistration-test.cpp)
target_link_libraries(tests/CollarLinesRegistration-test ${GTEST_LIBRARIES} pthread but_velodyne)

add_executable(tests/testsuite src/tests/testsuite.cpp)
target_link_libraries(tests/testsuite ${GTEST_LIBRARIES} pthread but_velodyne)

########################### Odometry estimation: #############################w

add_executable(collar-lines-odom src/apps/collar-lines-odom.cpp)
target_link_libraries(collar-lines-odom but_velodyne)

add_executable(collar-lines-registration src/apps/collar-lines-registration.cpp)
target_link_libraries(collar-lines-registration but_velodyne)

add_executable(register-dense-clouds src/apps/register-dense-clouds.cpp)
target_link_libraries(register-dense-clouds but_velodyne)

add_executable(overlaping-dense-clouds src/apps/overlaping-dense-clouds.cpp)
target_link_libraries(overlaping-dense-clouds but_velodyne)

add_executable(cls-reg-to-map src/apps/cls-reg-to-map.cpp)
target_link_libraries(cls-reg-to-map but_velodyne)

add_executable(cls-reg-subsequences src/apps/cls-reg-subsequences.cpp)
target_link_libraries(cls-reg-subsequences but_velodyne)

add_executable(transform-pcd src/apps/transform-pcd.cpp)
target_link_libraries(transform-pcd but_velodyne)

add_executable(overlaping-projections src/apps/overlaping-projections.cpp)
target_link_libraries(overlaping-projections but_velodyne)

add_executable(overlaping-projections-keyframes src/apps/overlaping-projections-keyframes.cpp)
target_link_libraries(overlaping-projections-keyframes but_velodyne)

add_executable(sync-velodyne-clouds src/apps/sync-velodyne-clouds.cpp)
target_link_libraries(sync-velodyne-clouds but_velodyne)

add_executable(calibrate-imu-velodyne-by-normals src/apps/calibrate-imu-velodyne-by-normals.cpp)
target_link_libraries(calibrate-imu-velodyne-by-normals but_velodyne)

add_executable(calibrate-imu-velodyne-manual src/apps/calibrate-imu-velodyne-manual.cpp)
target_link_libraries(calibrate-imu-velodyne-manual but_velodyne)

add_executable(calibrate-imu-velodyne-AXXB src/apps/calibrate-imu-velodyne-AXXB.cpp)
target_link_libraries(calibrate-imu-velodyne-AXXB but_velodyne)

add_executable(get-ranges src/apps/get-ranges.cpp)
target_link_libraries(get-ranges but_velodyne)

add_executable(rephase-velodyne-clouds src/apps/rephase-velodyne-clouds.cpp)
target_link_libraries(rephase-velodyne-clouds but_velodyne)

add_executable(interpolate-poses src/apps/interpolate-poses.cpp)
target_link_libraries(interpolate-poses but_velodyne)

add_executable(collar-lines-registration-line-clouds src/apps/collar-lines-registration-line-clouds.cpp)
target_link_libraries(collar-lines-registration-line-clouds but_velodyne)

add_executable(point-to-line-cloud src/apps/point-to-line-cloud.cpp)
target_link_libraries(point-to-line-cloud but_velodyne)

########################## Visualization apps: ###############################

add_executable(show-kitti-poses src/apps/show-kitti-poses.cpp)
target_link_libraries(show-kitti-poses but_velodyne)

add_executable(show-lcd-matches src/apps/show-lcd-matches.cpp)
target_link_libraries(show-lcd-matches but_velodyne)

add_executable(build-3d-model src/apps/build-3d-model.cpp)
target_link_libraries(build-3d-model but_velodyne)

add_executable(normalize-intensity src/apps/normalize-intensity.cpp)
target_link_libraries(normalize-intensity but_velodyne)

add_executable(cloud-intensities-normalization src/apps/cloud-intensities-normalization.cpp)
target_link_libraries(cloud-intensities-normalization but_velodyne)

add_executable(imu-orientations-extraction src/apps/imu-orientations-extraction.cpp)
target_link_libraries(imu-orientations-extraction but_velodyne)

########################## Loops detection/closure: ##########################

add_executable(sync-velodyne-clouds-by-angle src/apps/sync-velodyne-clouds-by-angle.cpp)
target_link_libraries(sync-velodyne-clouds-by-angle but_velodyne)

add_executable(filter-by-phase src/apps/filter-by-phase.cpp)
target_link_libraries(filter-by-phase but_velodyne)

add_executable(slampp-solution-to-poses src/apps/slampp-solution-to-poses.cpp)
target_link_libraries(slampp-solution-to-poses but_velodyne)

add_executable(correct-velodyne-distortion src/apps/correct-velodyne-distortion.cpp)
target_link_libraries(correct-velodyne-distortion but_velodyne)

add_executable(correct-velodyne-distortion-fine src/apps/correct-velodyne-distortion-fine.cpp)
target_link_libraries(correct-velodyne-distortion-fine but_velodyne)

add_executable(pose-graph-viewer src/apps/pose-graph-viewer.cpp)
target_link_libraries(pose-graph-viewer but_velodyne)

add_executable(correct-velodyne-distortion-per-parts src/apps/correct-velodyne-distortion-per-parts.cpp)
target_link_libraries(correct-velodyne-distortion-per-parts but_velodyne)

########################## Installation: #####################################

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/but_velodyne-config.cmake.in" "${CMAKE_CURRENT_BINARY_DIR}/but_velodyne-config.cmake" IMMEDIATE @ONLY)
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/but_velodyne-config-version.cmake.in" "${CMAKE_CURRENT_BINARY_DIR}/but_velodyne-config-version.cmake" IMMEDIATE @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/but_velodyne-config.cmake" "${CMAKE_CURRENT_BINARY_DIR}/but_velodyne-config-version.cmake"
    DESTINATION "share" COMPONENT ${COMPONENT_PREFIX}dev)

install(
TARGETS but_velodyne collar-lines-odom show-kitti-poses slampp-solution-to-poses
RUNTIME DESTINATION bin/
LIBRARY DESTINATION lib/
INCLUDES DESTINATION include/
)
