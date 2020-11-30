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
add_definitions(-D_POSIX_VER_64)
add_definitions(-ggdb3)

project(Demo)


file(GLOB CPP_SOURCE
        ./*.h
        ./*.cpp
        ./toolBox/*.h
        ./toolBox/*.cpp
)
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/" FILES ${CPP_SOURCE})
add_executable(Demo ${CPP_SOURCE})


#CEGUI
include_directories("/usr/include/cegui-0.8.7")
target_link_libraries(Demo -L/usr/local/lib -lCEGUIOgreRenderer-0 -lOgreMain_d -lpthread -lCEGUIBase-0)

#OIS
include_directories("/usr/include/ois")
target_link_libraries(Demo -lOIS)

#find_package(OGRE 1.9 REQUIRED)
#if(OGRE_FOUND)
    include_directories("/usr/local/include/OGRE" "/usr/local/include/OGRE/Bites" "/usr/local/include/OGRE/RTShaderSystem" "/usr/local/include/OGRE/Overlay")
    target_link_libraries(Demo -L/usr/local/lib -lOgreOverlay -lOgreRTShaderSystem -lOgreMain -L/usr/local/lib/OGRE -lzzip -lfreetyped -lboost_system)
#endif()



include_directories(./)


#Newton

include_directories(./toolBox)
include_directories(./glatter)
include_directories(/usr/include/dNewton/dMath/)
include_directories(/usr/include/dNewton/dgCore/)
include_directories(/usr/include/dNewton/dScene/)
include_directories(/usr/include/dNewton/dModel/)
include_directories(/usr/include/dNewton/dNewton/)
include_directories(/usr/include/dNewton/dgNewton/)
include_directories(/usr/include/dNewton/dVehicle/)
include_directories(/usr/include/dNewton/dgPhysics/)
include_directories(/usr/include/dNewton/dAnimation/)
include_directories(/usr/include/dNewton/dgMeshUtil/)
include_directories(/usr/include/dNewton/dContainers/)
include_directories(/usr/include/dNewton/dCustomJoints/)
include_directories(/usr/include/dNewton/thirdParty/tinyxml/)
include_directories(/usr/include/dNewton/dVehicle/dPlayerController)
include_directories(/usr/include/dNewton/dVehicle/dMultiBodyVehicle)

if (NEWTON_BUILD_PROFILER)
        include_directories(/usr/include/dNewton/dProfiler/dProfiler/)
endif ()

target_link_libraries (Demo dAnimation dContainers dMath newton dCustomJoints dNewton dScene dModel dVehicle -L../ -lglatter)
target_link_libraries (Demo GL GLU dl X11 pthread)

if (NEWTON_BUILD_PROFILER)
    target_link_libraries (Demo dProfiler)
endif ()

#to add ggdb3 : -DCMAKE_RELEASE_MODE=Debug
