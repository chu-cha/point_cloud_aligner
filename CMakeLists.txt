cmake_minimum_required(VERSION 3.15)  # Qt6 требует CMake 3.15+
project(aligner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(WIN32)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
	set(CMAKE_PREFIX_PATH "C:/vcpkg/installed/x64-windows")
	set(GLEW_ROOT "C:/vcpkg/installed/x64-windows")

    # set(CMAKE_TOOLCHAIN_FILE "C:/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE STRING "Vcpkg toolchain file")
else()
	set(CMAKE_PREFIX_PATH "/usr/lib/x86_64-linux-gnu/cmake/Qt6")
endif()

set(Qt5_DIR "" CACHE PATH "Empty Qt5 dir" FORCE)
set(QT_MAJOR_VERSION 6)
set(Qt5_DIR "Qt5_DIR-NOTFOUND")  # Явно сбрасываем поиск Qt5
set(QT_NO_AUTODETECT_QT5 ON)     # Запрещаем автоопределение Qt5

set(GLEW_USE_STATIC_LIBS OFF)
set(GLEW_LIBRARY "C:/vcpkg/installed/x64-windows/bin/glew32.dll")  # Для динамической версии
set(GLEW_INCLUDE_DIR "C:/vcpkg/installed/x64-windows/include")
find_package(GLEW REQUIRED)

if(GLEW_FOUND)
    message(STATUS "GLEW found: ${GLEW_INCLUDE_DIRS}, ${GLEW_LIBRARIES}")
else()
    message(FATAL_ERROR "GLEW not found. Install it via: 'vcpkg install glew:x64-windows' (Windows) or 'sudo apt-get install libglew-dev' (Linux).")
endif()

set(CMAKE_AUTOMOC ON)
set(QT_VERSION_MAJOR 6)  
find_package(Qt6 REQUIRED COMPONENTS Core Widgets OpenGLWidgets)

if(TARGET Qt5::Core)
    message(FATAL_ERROR "Qt5 still found! Please clean your build directory")
endif()


set(VTK_QT_VERSION 6)
set(VTK_GROUP_QT ON)
find_package(VTK REQUIRED COMPONENTS
    CommonCore
    InteractionStyle
    RenderingOpenGL2
	GUISupportQt
)



set(PCL_QT_VERSION 6)
find_package(PCL REQUIRED COMPONENTS common io visualization)

add_executable(aligner 
	main.cpp 
	main_window.cpp 
	main_window.h
	render_window.h
	render_window.cpp	)

target_link_libraries(aligner PRIVATE
    Qt6::Core 
    Qt6::Widgets
    Qt6::OpenGLWidgets 
    ${VTK_LIBRARIES}
    ${PCL_LIBRARIES}
    GLEW::GLEW
    pcl_visualization 
	VTK::GUISupportQt
)

message(STATUS "Using Qt version: ${Qt6_VERSION}")
message(STATUS "Using VTK version: ${VTK_VERSION}")
message(STATUS "Using PCL version: ${PCL_VERSION}")


get_cmake_property(_vars VARIABLES)
foreach(_var ${_vars})
    if(_var MATCHES "Qt5")
        message(STATUS "${_var}=${${_var}}")
    endif()
endforeach()

if(TARGET Qt5::Core)
    message(FATAL_ERROR "Обнаружен Qt5! Очистите кеш CMake и удалите все ссылки на Qt5")
endif()

# Копирование DLL на Windows
if(WIN32)
    #file(GLOB QT6_DLLS "C:/vcpkg/installed/x64-windows/bin/Qt6*.dll")
    #file(GLOB VTK_DLLS "C:/vcpkg/installed/x64-windows/bin/vtk*.dll")
	#file(GLOB PCL_DLLS "C:/vcpkg/installed/x64-windows/bin/pcl*.dll")
	#file(GLOB BOOST_DLLS "C:/vcpkg/installed/x64-windows/bin/boost*.dll")
file(GLOB DLL_FILES 
    "C:/vcpkg/installed/x64-windows/bin/pcl_*.dll"
    "C:/vcpkg/installed/x64-windows/bin/vtk*.dll"
	"C:/vcpkg/installed/x64-windows/bin/Qt6*.dll"
	"C:/vcpkg/installed/x64-windows/bin/*.dll"
)
add_custom_command(TARGET aligner POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${DLL_FILES}
    $<TARGET_FILE_DIR:aligner>
)
endif()

#Установка файлов (для пакетирования)
install(TARGETS aligner 
    RUNTIME DESTINATION bin
    BUNDLE DESTINATION .
)

# install(FILES 
    # ${CMAKE_SOURCE_DIR}/assets/icon.png
    # DESTINATION share/icons
    # RENAME aligner.png
# )

set(CPACK_GENERATOR "DEB") 
set(CPACK_PACKAGE_NAME "aligner")
set(CPACK_PACKAGE_FILE_NAME "aligner")  
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "t.o.nikolaeva@gmail.com")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Advanced point cloud processing application")

set(CPACK_SOURCE_GENERATOR "")  # Отключаем генерацию исходников
set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY FALSE)

set(CPACK_DEBIAN_PACKAGE_DEPENDS 
    "libqt6core6, libqt6gui6, libqt6widgets6, libvtk9.1, libpcl1.12")
set(CPACK_DEBIAN_PACKAGE_SECTION "utils")


include(CPack)