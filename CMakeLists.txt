cmake_minimum_required(VERSION 3.5.0)
project(kawai_airsim_controller)

add_compile_options(-std=c++11 -O2 -g -Wall)

find_package(OpenCV REQUIRED)
find_package(PCL)

## for "cnpy"
link_directories(/usr/local/lib)

# set(AIRSIM_ROOT ../AirSim)
find_path(AIRSIM_ROOT
	NAMES AirSim.sln 
	PATHS "./AirSim" "../AirSim" "../../AirSim" "../../../AirSim" "../../../../AirSim"
)
message(STATUS "found AIRSIM_ROOT=${AIRSIM_ROOT}")

add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib" AirLib)
add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" rpclib_wrapper)
add_subdirectory("${AIRSIM_ROOT}/cmake/MavLinkCom" MavLinkCom)

include_directories(
  include
	${AIRSIM_ROOT}/AirLib/include
	${AIRSIM_ROOT}/external/rpclib/rpclib-2.2.1/include
	${AIRSIM_ROOT}/MavLinkCom/include
	${AIRSIM_ROOT}/MavLinkCom/common_utils
	${AIRSIM_ROOT}/AirLib/deps/eigen3
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

add_executable(drone_random_pose src/drone_random_pose.cpp)
target_link_libraries(drone_random_pose AirLib rpc ${OpenCV_LIBRARIES} cnpy)

add_executable(random_pose_image src/random_pose_image.cpp)
target_link_libraries(random_pose_image AirLib rpc ${OpenCV_LIBRARIES} cnpy)

add_executable(get_drone_image src/get_drone_image.cpp)
target_link_libraries(get_drone_image AirLib rpc ${OpenCV_LIBRARIES} cnpy)

add_executable(original_image src/original_image.cpp)
target_link_libraries(original_image AirLib rpc ${OpenCV_LIBRARIES} cnpy)

add_executable(normal_distribution src/normal_distribution.cpp)
target_link_libraries(normal_distribution AirLib rpc ${OpenCV_LIBRARIES} cnpy)

add_executable(get_inference_image src/get_inference_image.cpp)
target_link_libraries(get_inference_image AirLib rpc ${OpenCV_LIBRARIES} cnpy)