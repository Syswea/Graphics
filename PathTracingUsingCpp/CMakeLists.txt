# 最低版本要求
cmake_minimum_required(VERSION 3.28)

# 项目
project(Renderer)

# 编译器指定
set(CMAKE_C_COMPILER "/usr/bin/gcc")
set(CMAKE_CXX_COMPILER "/usr/bin/g++")

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_CXX_FLAGS "${CAMKE_CXX_FLAGS} -O3 -fopenmp")
set(CMAKE_BUILD_TYPE Release)

# 查找源文件
file(GLOB src
    "src/*.cpp"
)

# 生成可执行文件
add_executable(Renderer ${src})

# 头文件目录
# target_include_directories(Renderer PRIVATE
#     "C:/Libs/vulkan/Include"
#     "C:/Libs/glm"
#     "C:/Libs/glfw/include"
# )

# 链接库文件
# target_link_libraries(Renderer PRIVATE
#     # "C:/Libs/vulkan/Lib/vulkan-1.lib"
#     # "C:/Libs/glfw/lib-mingw-w64/libglfw3.a"
#     glfw vulkan dl pthread X11 Xxf86vm Xrandr Xi
# )

# 创建自定义目标 'test'
add_custom_target(test
    COMMAND $<TARGET_FILE:Renderer>
    DEPENDS Renderer
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Running the Renderer executable for testing"
)
