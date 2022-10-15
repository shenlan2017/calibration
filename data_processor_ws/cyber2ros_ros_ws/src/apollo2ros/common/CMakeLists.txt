project (common)

set(GLOG_LIB glog)

file(GLOB _SRCS "*.cc" "*.h" "*.hpp")
add_library(${PROJECT_NAME} SHARED ${_SRCS})

target_include_directories(${PROJECT_NAME} PUBLIC 
    ${PROJECT_SOURCE_DIR}
)
target_link_libraries(${PROJECT_NAME}
    gflags
    glog
)

