cmake_minimum_required(VERSION 3.20)
project(space_docking_system VERSION 1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_definitions(_USE_MATH_DEFINES)

# Timing matters here: default to an optimised build when none is chosen
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(Threads REQUIRED)

# Dependencies
include(FetchContent)

# GLFW
FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG        3.3.8
)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(glfw)

# GLM
FetchContent_Declare(
    glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG        6f14f4792a0cde5d0cf2c910506724d61cb95834   # pinned (was: master)
)
FetchContent_MakeAvailable(glm)

# GLAD library (static)
add_library(glad third_party/glad/src/glad.c)
target_include_directories(glad PUBLIC third_party/glad/include)
target_link_libraries(glad PUBLIC ${CMAKE_DL_LIBS})   # glad.c uses dlopen on Linux

# ImGui
FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        64944b4520b30772de8dbf0b37d0311746477a32   # pinned (was: docking)
)
FetchContent_MakeAvailable(imgui)

add_library(imgui_lib STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui_lib PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
)
target_link_libraries(imgui_lib PUBLIC glfw glad)

# Sim Executable
add_executable(sim sim.cpp)
target_link_libraries(sim PRIVATE glad glfw glm imgui_lib Threads::Threads)

if (APPLE)
    target_link_libraries(sim PRIVATE "-framework OpenGL" "-framework Cocoa" "-framework IOKit" "-framework CoreVideo")
    target_compile_definitions(sim PRIVATE GL_SILENCE_DEPRECATION)
elseif (WIN32)
    target_link_libraries(sim PRIVATE opengl32 winmm)
    if (MINGW)
        target_link_libraries(sim PRIVATE -static -static-libgcc -static-libstdc++)
    endif()
else()
    find_package(OpenGL REQUIRED)
    target_link_libraries(sim PRIVATE OpenGL::GL)
endif()


