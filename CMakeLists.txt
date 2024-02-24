# cmake needs this line
cmake_minimum_required(VERSION 3.1)

set(PROJECT_NAME cyclops)
# Define project name
project(${PROJECT_NAME})

# OpenCV
find_package(OpenCV 4.8 REQUIRED)
message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

# CUDA
#find_package(CUDAToolkit)
#if(CUDAToolkit_FOUND)
#	add_compile_definitions(WITH_CUDA)
#	message(STATUS "CUDA found, assuming OpenCV was built with CUDA")
#endif(CUDAToolkit_FOUND)

# OpenGL
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(glm REQUIRED)

# Json
add_subdirectory("thirdparty/json")

# Coral / EdgeTPU
find_library(edgetpu_FOUND edgetpu)
if(edgetpu_FOUND)
	add_subdirectory(
		"thirdparty/tensorflow/tensorflow/lite"
		"${CMAKE_CURRENT_BINARY_DIR}/tensorflow-lite" EXCLUDE_FROM_ALL)
	set(CORALLIBS tensorflow-lite edgetpu)
	message("EdgeTPU library installed on system, including tensorflow...")
else(edgetpu_FOUND)
	message("EdgeTPU library not installed on system, not including tensorflow...")
endif(edgetpu_FOUND)

# Auto include subfolders
MACRO(HEADER_DIRECTORIES return_list)
	FILE(GLOB_RECURSE new_list include/*.h*)
	SET(dir_list "")
	FOREACH(file_path ${new_list})
		GET_FILENAME_COMPONENT(dir_path ${file_path} PATH)
		SET(dir_list ${dir_list} ${dir_path})
	ENDFOREACH()
	LIST(REMOVE_DUPLICATES dir_list)
	SET(${return_list} ${dir_list})
ENDMACRO()

HEADER_DIRECTORIES(INCLUDES)

# Imgui files
set(imguiroot ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/imgui)
set(imguifolders ${imguiroot} ${imguiroot}/backends)

set(imguisource ${imguiroot}/imgui.cpp ${imguiroot}/imgui_tables.cpp ${imguiroot}/imgui_widgets.cpp ${imguiroot}/imgui_draw.cpp ${imguiroot}/imgui_demo.cpp)
set(imguisource ${imguisource} ${imguiroot}/backends/imgui_impl_glfw.cpp ${imguiroot}/backends/imgui_impl_opengl3.cpp)

add_compile_definitions(IMGUI_USER_CONFIG="Visualisation/imguiconfig.hpp")

# Extra compiler options
add_compile_options(-Wall -Wextra -Wpedantic)

message(STATUS "include directories : ${INCLUDES}")

include_directories(
	${OPENCV_INCLUDE_DIRS}
	${INCLUDES}
	${OPENGL_INCLUDE_DIRS}
	${GLUT_INCLUDE_DIRS}
	${GLEW_INCLUDE_DIRS}
	${imguifolders})

file(GLOB_RECURSE SOURCES "source/*.cpp")
set(SOURCES ${SOURCES} ${imguisource})

add_executable(${PROJECT_NAME} ${SOURCES})


#message(STATUS "Sources found : ${SOURCES}")

set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 17)
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")

# Link your application with OpenCV libraries

target_link_libraries(${PROJECT_NAME} 
	${OpenCV_LIBS} 
	stdc++fs
	config++
	${OPENGL_LIBRARIES} 
	glfw
	GLEW::glew
	glm::glm
	assimp
	nlohmann_json::nlohmann_json
	${CORALLIBS}
)