cmake_minimum_required(VERSION 3.1.0)
project(practica2)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
ADD_DEFINITIONS( -Wall -DQT_DLL -DQT_GUI_LIB -DQT_CORE_LIB )
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++11")
set(SOURCES main.cpp
            computepointcloud/computepointcloud.cpp
            mainwindow.cpp
            osgview.cpp
            Luz/luz.cpp
            viewer/viewer.cpp
            osgWay/osgWay.cpp
            processfunction/processfunction.cpp
            descriptors/descriptors.cpp )

set(CMAKE_CXX_STANDARD 14)


#
# Qt5
#
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
find_package(Qt5Widgets)
find_package(Qt5OpenGL)
SET( QT_USE_QTGUI TRUE )
SET( QT_USE_QTOPENGL TRUE )
include_directories(${CMAKE_CURRENT_BINARY_DIR} ${QT_QTCORE_INCLUDE_DIR} ${QT_QTCORE_INCLUDE_DIR}/.. ${QT_QTGUI_INCLUDE_DIR} )

#
# OpenGL
#
find_package(OpenGL)

#
# OpenCV 2
#
find_package(OpenCV 3 REQUIRED)
INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIR} )
SET(LIBS ${LIBS} ${OpenCV_LIBS})
MESSAGE(STATUS ${OpenCV_LIBS})



#
# OpenSceneGraph
#
INCLUDE_DIRECTORIES( /usr/local/include/osg /usr/include/osgViewer )
SET(LIBS ${OSG_LIBS} -L/usr/local/lib64/ -losg -losgViewer -losgGA -losgText -losgDB -losgUtil -fopenmp)


#
# PointCloudLibrary
#
find_package(PCL 1.7 REQUIRED )
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
# ADD_DEFINITIONS( -std=c++11 )
ADD_DEFINITIONS( -fopenmp)

#
# Link
#
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
add_executable(practica2 ${SOURCES} )
qt5_use_modules(practica2 Widgets OpenGL)
target_link_libraries(practica2 ${LIBS} ${PCL_LIBRARIES} ${QT_LIBRARIES} ${OPENGL_LIBRARIES} libapriltags.a -lopencv_highgui ${OpenCV_LIBS} )
