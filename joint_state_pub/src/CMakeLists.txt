cmake_minimum_required(VERSION 2.8)
project(jaka_vision)

#set(CMAKE_MODULE_PATH "/usr/share/cmake-3.2/Modules" "./cmake_modules")
message(STATUS "*************************CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH} "************************")

set(CMAKE_BUILD_TYPE "Debug")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -Wall -Wno-unused-variable -pthread -fopenmp")

find_package(OpenCV 3.0 REQUIRED)
find_package(Eigen3 REQUIRED)

# Assumes that gRPC and all its dependencies are already installed
# on this system, so they can be located by find_package().

# Find Protobuf installation
# Looks for protobuf-config.cmake file installed by Protobuf's cmake installation.
set(protobuf_MODULE_COMPATIBLE TRUE)
find_package(Protobuf CONFIG REQUIRED)
message(STATUS "Using protobuf ${protobuf_VERSION}")

set(_PROTOBUF_LIBPROTOBUF protobuf::libprotobuf)
set(_PROTOBUF_PROTOC $<TARGET_FILE:protobuf::protoc>)

# Find gRPC installation
# Looks for gRPCConfig.cmake file installed by gRPC's cmake installation.
find_package(gRPC CONFIG REQUIRED)
message(STATUS "Using gRPC ${gRPC_VERSION}")

set(_GRPC_GRPCPP_UNSECURE gRPC::grpc++_unsecure)
set(_GRPC_CPP_PLUGIN_EXECUTABLE $<TARGET_FILE:gRPC::grpc_cpp_plugin>)

# Proto file
get_filename_component(jk_proto "./jaka.proto" ABSOLUTE)
get_filename_component(jk_proto_path "${jk_proto}" PATH)

# Generated sources
set(jk_proto_srcs "${CMAKE_CURRENT_BINARY_DIR}/jaka.pb.cc")
set(jk_proto_hdrs "${CMAKE_CURRENT_BINARY_DIR}/jaka.pb.h")
set(jk_grpc_srcs "${CMAKE_CURRENT_BINARY_DIR}/jaka.grpc.pb.cc")
set(jk_grpc_hdrs "${CMAKE_CURRENT_BINARY_DIR}/jaka.grpc.pb.h")
add_custom_command(
      OUTPUT "${jk_proto_srcs}" "${jk_proto_hdrs}" "${jk_grpc_srcs}" "${jk_grpc_hdrs}"
      COMMAND ${_PROTOBUF_PROTOC}
      ARGS --grpc_out "${CMAKE_CURRENT_BINARY_DIR}"
        --cpp_out "${CMAKE_CURRENT_BINARY_DIR}"
        -I "${jk_proto_path}"
        --plugin=protoc-gen-grpc="${_GRPC_CPP_PLUGIN_EXECUTABLE}"
        "${jk_proto}"
      DEPENDS "${jk_proto}")

# Include generated *.pb.h files
include_directories("${CMAKE_CURRENT_BINARY_DIR}")

message("---------------------------" ${_GRPC_GRPCPP_UNSECURE} "    "   ${_PROTOBUF_LIBPROTOBUF})

set(DEPENDENCIES
    realsense2
    ${OpenCV_LIBS}
    ${_GRPC_GRPCPP_UNSECURE}
    ${_PROTOBUF_LIBPROTOBUF})

## Targets greeter_[async_](client|server)
#foreach(_target
#  greeter_client greeter_server
#  greeter_async_client greeter_async_server)
#  add_executable(${_target} "${_target}.cc"
#    ${jk_proto_srcs}
#    ${jk_grpc_srcs})
#  target_link_libraries(${_target}
#    ${_GRPC_GRPCPP_UNSECURE}
#    ${_PROTOBUF_LIBPROTOBUF})
#endforeach()

#add_library(pose_watcher SHARED
#    src/watch_pose_info.h
#    src/watch_pose_info.cpp
#    )
#target_link_libraries(pose_watcher ${catkin_LIBRARIES})

#add_executable(main_test_watch_pose_info src/main_test_watch_pose_info.cpp)
#target_link_libraries(main_test_watch_pose_info pose_watcher ${catkin_LIBRARIES})

#include_directories( ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
#set(DEPENDENCIES realsense2 ${OpenCV_LIBS})

#add_executable(main_save_image_and_pose src/main_save_image_and_pose.cpp)
#target_link_libraries(main_save_image_and_pose
#    pose_watcher
#    ${DEPENDENCIES}
#    ${catkin_LIBRARIES}
#    )

add_library(robot SHARED
    src/robot.h
    src/robot.cpp
    src/robot_client.cc
    src/robot_client.h
    ${jk_proto_srcs}
    ${jk_grpc_srcs}
    )
    
#add_executable(main_fetch src/main_fetch.cpp
#                            src/move.cpp)
#target_link_libraries(main_fetch
#	robot
#    pose_watcher
#    ${DEPENDENCIES}
#    ${catkin_LIBRARIES}
#    )

add_executable(main_handeye_calib
    src/main_handeye_calib.cpp
    src/handeye.cpp)
target_link_libraries(main_handeye_calib
    robot
    ${DEPENDENCIES}
    )

add_executable(main_save_image
    src/main_save_image_and_pose.cpp
    )
target_link_libraries(main_save_image
    robot
    ${DEPENDENCIES}
    )


add_executable(main_jaka_test
    src/main_jaka_test.cpp)
target_link_libraries(main_jaka_test
    robot
    ${DEPENDENCIES}
    )

add_executable(main_debug
    src/main_debug.cpp
    src/handeye.cpp)
target_link_libraries(main_debug
    robot
    ${DEPENDENCIES}
    )
