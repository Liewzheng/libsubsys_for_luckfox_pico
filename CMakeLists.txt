cmake_minimum_required(VERSION 3.16)

# 项目信息
project(libsubsys
    VERSION 1.0.0
    DESCRIPTION "Subsystem Communication Library"
    LANGUAGES C
)

# 设置 C 标准
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# 如果是作为子项目构建，使用父项目的输出目录
if(DEFINED CMAKE_LIBRARY_OUTPUT_DIRECTORY_FOR_SUBMODULES)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY_FOR_SUBMODULES})
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY_FOR_SUBMODULES})
endif()

# 头文件目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# 源文件
set(SUBSYS_SOURCES
    source/subsys.c
)

# 头文件
set(SUBSYS_HEADERS
    include/subsys.h
)

# 创建动态库
add_library(subsys SHARED ${SUBSYS_SOURCES})

# 设置库的属性
set_target_properties(subsys PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
    PUBLIC_HEADER "${SUBSYS_HEADERS}"
)

# 编译选项
target_compile_options(subsys PRIVATE
    -Wall
    -Wextra
    -Wno-unused-parameter
)

# 链接系统库
target_link_libraries(subsys PRIVATE
    pthread
    m
)

# 调试模式下启用调试输出
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_definitions(subsys PRIVATE DEBUG=1)
endif()

# 安装规则
install(TARGETS subsys
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    PUBLIC_HEADER DESTINATION include
)

# 创建 pkg-config 文件
configure_file(libsubsys.pc.in libsubsys.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/libsubsys.pc DESTINATION lib/pkgconfig)

# 显示构建信息
message(STATUS "libsubsys 配置完成:")
message(STATUS "  版本: ${PROJECT_VERSION}")
message(STATUS "  源文件: ${SUBSYS_SOURCES}")
message(STATUS "  输出目录: ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
