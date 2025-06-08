cmake_minimum_required(VERSION 3.5)
project(aligner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(WIN32)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
    set(CMAKE_PREFIX_PATH "C:/vcpkg/installed/x64-windows")
    set(GLEW_ROOT "C:/vcpkg/installed/x64-windows")
	set(VTK_DIR "C:/externals/VTK-9.4.2/install/lib/cmake/vtk-9.4")
	set(PCL_DIR "C:/externals/pcl_x64-windows_working ver/share/pcl")
	
	add_compile_options(/arch:AVX)  # для AVX
	add_compile_options(/arch:AVX2) # для AVX2
else()
    set(CMAKE_PREFIX_PATH "/usr/lib/x86_64-linux-gnu/cmake/Qt5")
    set(VTK_DIR "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.3") 
endif()

set(QT_MAJOR_VERSION 5)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets OpenGL)

set(GLEW_USE_STATIC_LIBS OFF)
find_package(GLEW REQUIRED)

if(GLEW_FOUND)
    message(STATUS "GLEW found: ${GLEW_INCLUDE_DIRS}, ${GLEW_LIBRARIES}")
else()
    message(FATAL_ERROR "GLEW not found.")
endif()

set(CMAKE_AUTOMOC ON)
find_package(VTK REQUIRED COMPONENTS
    CommonCore
    InteractionStyle
    RenderingOpenGL2
    GUISupportQt
)

if(NOT VTK_FOUND)
  message(FATAL_ERROR "VTK not found!")
endif()

find_package(PCL REQUIRED COMPONENTS common io visualization)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_executable(aligner 
    main.cpp 
    main_window.cpp 
    main_window.h
    render_window.h
    render_window.cpp)

target_link_libraries(aligner PRIVATE
    Qt5::Core 
    Qt5::Widgets
    Qt5::OpenGL
    ${VTK_LIBRARIES}
    ${PCL_LIBRARIES}
    GLEW::GLEW
    pcl_visualization 
    VTK::GUISupportQt
)

add_definitions(-DPCL_SILENCE_MALLOC_WARNING=1)

message(STATUS "Using Qt version: ${Qt5_VERSION}")
message(STATUS "Using VTK version: ${VTK_VERSION}")
message(STATUS "Using PCL version: ${PCL_VERSION}")

if(WIN32)
    file(GLOB DLL_FILES 
        "C:/externals/pcl_x64-windows_working ver/bin/pcl_*.dll"
        "C:/externals/VTK-9.4.2/install/bin/vtk*.dll"
        "C:/vcpkg/installed/x64-windows/bin/Qt5*.dll"
        "C:/vcpkg/installed/x64-windows/bin/*.dll"
    )
    add_custom_command(TARGET aligner POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${DLL_FILES}
        $<TARGET_FILE_DIR:aligner>
    )
endif()

install(TARGETS aligner 
    RUNTIME DESTINATION bin
    BUNDLE DESTINATION .
)

set(CPACK_GENERATOR "DEB") 
set(CPACK_PACKAGE_NAME "aligner")
set(CPACK_PACKAGE_FILE_NAME "aligner")  
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "t.o.nikolaeva@gmail.com")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Advanced point cloud processing application")

set(CPACK_DEBIAN_PACKAGE_DEPENDS 
    "qtbase5-dev, libqt5opengl5-dev, libvtk9-dev, libpcl-dev")
set(CPACK_DEBIAN_PACKAGE_SECTION "utils")

include(CPack)