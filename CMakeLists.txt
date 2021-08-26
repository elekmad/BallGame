# Copyright (c) <2014-2017> <Newton Game Dynamics>
#
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
#
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely.

cmake_minimum_required(VERSION 3.4.0)
set (CMAKE_CXX_STANDARD 11)
add_definitions(-D_POSIX_VER_64)
add_definitions(-ggdb3)

project(Editor)


file(GLOB CPP_SOURCE
        ./*.h
        ./*.cpp
        ./toolBox/*.h
        ./toolBox/*.cpp
)
include_directories("./OgreOverlay")

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/" FILES ${CPP_SOURCE})
add_executable(Editor ${CPP_SOURCE})

find_package(PkgConfig REQUIRED)

#RapidJson
pkg_check_modules(Json REQUIRED IMPORTED_TARGET RapidJSON)
target_link_libraries(Editor ${Json_LIBRARIES})
include_directories(${Json_INCLUDE_DIRS})

#CEGUI
pkg_check_modules(CEGUI REQUIRED IMPORTED_TARGET CEGUI-0-OGRE)
target_link_libraries(Editor ${CEGUI_LIBRARIES})
include_directories(${CEGUI_INCLUDE_DIRS})

#OIS
pkg_check_modules(OIS REQUIRED IMPORTED_TARGET OIS)
target_link_libraries(Editor ${OIS_LIBRARIES})
include_directories(${OIS_INCLUDE_DIRS})

#pkg_check_modules(OGRE REQUIRED IMPORTED_TARGET OGRE)

set(OGRE_BASE_REP "/usr/local")

include_directories("${OGRE_BASE_REP}/include/OGRE" "${OGRE_BASE_REP}/include/OGRE/Bites" "${OGRE_BASE_REP}/include/OGRE/RTShaderSystem" "${OGRE_BASE_REP}/include/OGRE/Overlay")
target_link_libraries(Editor -L${OGRE_BASE_REP}/lib -lOgreOverlay -lOgreRTShaderSystem -lOgreMain -L${OGRE_BASE_REP}/lib/OGRE -lzzip -lfreetype -lboost_system)



include_directories(./)


#Newton

include_directories(./toolBox)
include_directories(./glatter)
include_directories(/usr/local/include/newton/)
include_directories(/usr/local/include/dMath/)
include_directories(/usr/local/include/dgCore/)
include_directories(/usr/local/include/dScene/)
include_directories(/usr/local/include/dModel/)
include_directories(/usr/local/include/dNewton/)
include_directories(/usr/local/include/dgNewton/)
include_directories(/usr/local/include/dVehicle/)
include_directories(/usr/local/include/dgPhysics/)
include_directories(/usr/local/include/dAnimation/)
include_directories(/usr/local/include/dgMeshUtil/)
include_directories(/usr/local/include/dContainers/)
include_directories(/usr/local/include/dCustomJoints/)
include_directories(/usr/local/include/thirdParty/tinyxml/)
include_directories(/usr/local/include/dVehicle/dPlayerController)
include_directories(/usr/local/include/dVehicle/dMultiBodyVehicle)

if (NEWTON_BUILD_PROFILER)
        include_directories(/usr/local/include/dNewton/dProfiler/dProfiler/)
endif ()

target_link_libraries (Editor dAnimation dContainers dMath newton dCustomJoints dNewton dScene dModel dVehicle -L../ -lglatter)
target_link_libraries (Editor GL GLU dl X11 pthread)

if (NEWTON_BUILD_PROFILER)
    target_link_libraries (Editor dProfiler)
endif ()

#to add ggdb3 : -DCMAKE_RELEASE_MODE=Debug
