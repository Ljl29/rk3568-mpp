cmake_minimum_required(VERSION 2.8)
project(v4l2_mpp_rtsp C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -O2 -g -std=gnu99")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O2 -g")

# MPP public headers
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../../inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../osal/inc
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# live555 (self-compiled, aarch64 cross)
set(LIVE555_ROOT /home/ljl/tools/live555/build)
include_directories(
    ${LIVE555_ROOT}/include/liveMedia
    ${LIVE555_ROOT}/include/groupsock
    ${LIVE555_ROOT}/include/BasicUsageEnvironment
    ${LIVE555_ROOT}/include/UsageEnvironment
)
link_directories(${LIVE555_ROOT}/lib)

add_executable(v4l2_mpp_rtsp
    main.c
    v4l2_capture.c
    mpp_encoder.c
    ts_queue.c
    H264FramedSource.cpp
    rtsp_server.cpp
)

target_link_libraries(v4l2_mpp_rtsp
    rockchip_mpp
    liveMedia
    groupsock
    BasicUsageEnvironment
    UsageEnvironment
    pthread
)

install(TARGETS v4l2_mpp_rtsp RUNTIME DESTINATION bin)
