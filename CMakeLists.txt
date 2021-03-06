cmake_minimum_required(VERSION 3.2)
project(SAP)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -ffast-math" )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DGLM_FORCE_RADIANS -DGLM_PRECISION_MEDIUMP_FLOAT" )

IF (CMAKE_BUILD_TYPE STREQUAL "Release")
    #disable profiling
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DNPROFILE -Wall -Werror -Wno-strict-aliasing" )
ENDIF()

IF (CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    IF (WIN32)
        # for profiling
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-omit-frame-pointer -gdwarf-2" )
    ENDIF()
ENDIF()

IF (WIN32)
    include_directories(c:/DEV/ENV/msys64/mingw64/include/)
    link_directories(c:/DEV/ENV/msys64/mingw64/lib/)
    include_directories(c:/DEV/gamedev/base/include/)
    include_directories(c:/DEV/gamedev/maths/include/)
    include_directories(c:/DEV/gamedev/assets/include/)
ELSE()
    include_directories(/home/grynca/DEV/gamedev/base/include/)
    include_directories(/home/grynca/DEV/gamedev/maths/include/)
    include_directories(/home/grynca/DEV/gamedev/assets/include/)
ENDIF()

option(USE_SDL2 "Use SDL2" ON)
IF (USE_SDL2)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_SDL2=1" )
    IF (WIN32)
        set(LIBS "-lglew32 -lglu32 -lopengl32 -lmingw32 -lSDL2main -lSDL2 -lSDL2_image")
    ELSE()
        set(LIBS "-lGLEW -lGL -lGLU -lSDL2 -lSDL2_image")
    ENDIF()
ENDIF()

include_directories(include/)

set(INCL_FILES
        include/SAP/SAP_domain.h
        include/SAP/SAPManagerC.h
        include/SAP/SAPManagerC.inl
        include/SAP/SAP_internal.h
        include/SAP/SAP_internal.inl
        include/SAP.h
        include/SAP/SAPSegment.h
        include/SAP/SAPSegment.inl
        include/SAP/SAP_config.h
        include/SAP/SAPRaycaster.h
        include/SAP/SAPRaycaster.inl
        )
set(SRC_FILES
        test/main.cpp
        test/SAP_test.h)
add_executable(SAP ${INCL_FILES} ${SRC_FILES})
target_link_libraries(SAP ${LIBS})
