cmake_minimum_required(VERSION 3.2)
project(SAP)

option(VISUAL_DEBUG "Visual Debugging" ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -std=gnu++0x" )
IF (VISUAL_DEBUG)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DVISUAL_DEBUG" )
ENDIF()

IF (WIN32)
    # for profiling
    #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -fno-omit-frame-pointer" )

    include_directories(c:/DEV/libs/mingw/glm/)
    include_directories(c:/DEV/gamedev/base/src/)
    include_directories(c:/DEV/gamedev/maths/include/)
    IF (VISUAL_DEBUG)
        include_directories(c:/DEV/gamedev/assets/include/)
        include_directories(c:/DEV/libs/mingw/SDL2-2.0.4/x86/include/)
        link_directories(c:/DEV/libs/mingw/SDL2-2.0.4/x86/lib)
        include_directories(c:/DEV/libs/mingw/glew-1.13.0/include/)
        link_directories(c:/DEV/libs/mingw/glew-1.13.0/lib/)

        set(LIBS "-lglew32 -lglu32 -lopengl32 -lmingw32 -lSDL2main -lSDL2 -lSDL2_image")
    ENDIF()
ELSE()
    include_directories(/home/grynca/DEV/gamedev/base/src/)
    include_directories(/home/grynca/DEV/gamedev/maths/include/)
    include_directories(/home/grynca/DEV/gamedev/assets/include/)

    set(LIBS "-lGLEW -lGL -lGLU -lfreetype -lSDL2 -lSDL2_image")
ENDIF()

set(INCL_FILES
        include/SAP/SAPManager.h
        include/SAP/SAPManager.inl
        include/SAP/SAP_internal.h
        include/SAP/SAP_internal.inl
        include/SAP.h
        include/SAP/SAPSegment.h
        include/SAP/SAPSegment.inl
        include/SAP/SAP_config.h
        include/SAP/PairManager.h
        include/SAP/PairManager.inl
        include/SAP/SAPRaycaster.h
        include/SAP/SAPRaycaster.inl
    )
set(SRC_FILES
        src/main.cpp
        src/update_loop.h
        src/update_loop.cpp
        src/test_cfg.h)
add_executable(SAP ${INCL_FILES} ${SRC_FILES})
target_link_libraries(SAP ${LIBS})
