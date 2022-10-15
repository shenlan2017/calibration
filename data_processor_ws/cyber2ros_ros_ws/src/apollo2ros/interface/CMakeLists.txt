project (interface)

file(GLOB _SRCS "*.cc" "*.cpp" "*.h")
add_library(${PROJECT_NAME} SHARED ${_SRCS})

target_include_directories(${PROJECT_NAME} PUBLIC
    ${PROJECT_SOURCE_DIR}
    ${YAML_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
    common
)

