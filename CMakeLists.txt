# cmake needs this line
cmake_minimum_required(VERSION 3.1)

set(PROJECT_NAME cyclops)
# Define project name
project(${PROJECT_NAME})

# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
find_package(OpenCV 4.7 REQUIRED)

#Window manager, used to get the screen resolution
find_package(X11)
if(${X11_FOUND})
	add_compile_definitions(WITH_X11)
endif(${X11_FOUND})

#find_package(CUDAToolkit)
#if(CUDAToolkit_FOUND)
#	add_compile_definitions(WITH_CUDA)
#	message(STATUS "CUDA found, assuming OpenCV was built with CUDA")
#endif(CUDAToolkit_FOUND)

# OpenGL
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(glm REQUIRED)

add_subdirectory("thirdparty/json")

# If the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of them
message(STATUS "Cmake source dir : ${CMAKE_SOURCE_DIR}")
message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

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

#imgui files
set(imguiroot ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/imgui)
set(imguifolders ${imguiroot} ${imguiroot}/backends)

set(imguisource ${imguiroot}/imgui.cpp ${imguiroot}/imgui_tables.cpp ${imguiroot}/imgui_widgets.cpp ${imguiroot}/imgui_draw.cpp ${imguiroot}/imgui_demo.cpp)
set(imguisource ${imguisource} ${imguiroot}/backends/imgui_impl_glfw.cpp ${imguiroot}/backends/imgui_impl_opengl3.cpp)

add_compile_definitions(IMGUI_USER_CONFIG="visualisation/imguiconfig.hpp")

#message(STATUS "include directories : ${INCLUDES}")

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
	${X11_LIBRARIES}
	Xrandr
	config++
	${OPENGL_LIBRARIES} 
	glfw
	GLEW::glew
	glm::glm
	assimp
	nlohmann_json::nlohmann_json
)