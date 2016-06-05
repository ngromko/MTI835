cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
cmake_policy(VERSION 2.8)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

set(NAME MTI835-NG)

project(${NAME})
find_package(OpenCL REQUIRED)
include_directories(external)
include_directories(headers)
include_directories(external/glm)
include_directories(external/gli)
include_directories(external/assimp)
include_directories(external/bullet3)
include_directories(base)
include_directories(${OpenCL_INCLUDE_DIRS})

IF(WIN32)
	find_library(VULKAN_LIB NAMES vulkan-1 vulkan PATHS ${CMAKE_SOURCE_DIR}/libs/vulkan)
	find_library(ASSIMP_LIBRARIES NAMES assimp libassimp.dll.a PATHS ${CMAKE_SOURCE_DIR}/libs/assimp)
        #find_library(BULLET_LIBRAIRIES NAMES BulletDynamics PATHS ${CMAKE_SOURCE_DIR}/libs/bullet3)
        #find_library(BULLET_LIBRAIRIES1 NAMES BulletCollision PATHS ${CMAKE_SOURCE_DIR}/libs/bullet3)
        #find_library(BULLET_LIBRAIRIES2 NAMES LinearMath PATHS ${CMAKE_SOURCE_DIR}/libs/bullet3)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DVK_USE_PLATFORM_WIN32_KHR")
ELSE(WIN32)
	find_library(VULKAN_LIB NAMES libvulkan.so PATHS ${CMAKE_SOURCE_DIR}/libs/vulkan)
	find_package(ASSIMP REQUIRED)
	find_package(XCB REQUIRED)
	find_package(Threads REQUIRED)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DVK_USE_PLATFORM_XCB_KHR")
	# Todo : android?
ENDIF(WIN32)

message(STATUS ${VULKAN_LIB})
message(STATUS ${BULLET_LIBRAIRIES})
message(STATUS ${BULLET_LIBRAIRIES1})
message(STATUS ${BULLET_LIBRAIRIES2})

# Set preprocessor defines
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DNOMINMAX -D_USE_MATH_DEFINES")

add_definitions(-D_CRT_SECURE_NO_WARNINGS)
add_definitions(-std=c++14)
add_definitions(-std=c++0x)
add_definitions(-std=gnu++14)

file(GLOB SOURCE *.cpp base/*.cpp)

# Function for building single example
function(buildExample EXAMPLE_NAME)
        file(GLOB SOURCE *.cpp base/*.cpp src/*.cpp)
        file(GLOB_RECURSE bullet external/bullet3/*.cpp)
        file(GLOB_RECURSE headers headers/*.h)
	if(WIN32)
                add_executable(${EXAMPLE_NAME} WIN32 src/${EXAMPLE_NAME}.cpp ${SOURCE} ${bullet} ${headers})
                target_link_libraries(${EXAMPLE_NAME} ${VULKAN_LIB} ${ASSIMP_LIBRARIES} ${WINLIBS} ${OpenCL_LIBRARIES})
	else(WIN32)
                add_executable(${EXAMPLE_NAME} src/${EXAMPLE_NAME}.cpp ${SOURCE})
                target_link_libraries(${EXAMPLE_NAME} ${VULKAN_LIB} ${ASSIMP_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} $(BULLET_LIBRAIRIES) $(BULLET_LIBRAIRIES1) $(BULLET_LIBRAIRIES2))
	endif(WIN32)
endfunction(buildExample)

# Build all examples
function(buildExamples)
	foreach(EXAMPLE ${EXAMPLES})
		buildExample(${EXAMPLE})
    endforeach(EXAMPLE)
endfunction(buildExamples)

# Compiler specific stuff
IF(MSVC)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc")
ENDIF(MSVC)

IF(WIN32)
	# Nothing here (yet)
ELSE(WIN32)
	link_libraries(${XCB_LIBRARIES} ${VULKAN_LIB})
ENDIF(WIN32)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "bin/")

set(EXAMPLES 
        main
)

buildExamples()
