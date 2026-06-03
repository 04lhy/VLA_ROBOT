find_package(Protobuf REQUIRED)

set(msgs grasp_event.proto)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})

add_library(gazebo_grasp_msgs SHARED ${PROTO_SRCS})
target_include_directories(gazebo_grasp_msgs PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(gazebo_grasp_msgs ${PROTOBUF_LIBRARIES})

install(TARGETS gazebo_grasp_msgs
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/
  DESTINATION include/gazebo_grasp_plugin
  FILES_MATCHING PATTERN "*.pb.h"
)
