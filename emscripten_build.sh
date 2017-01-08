#!/bin/bash
mkdir -p build
cd build
mkdir -p Emscripten
cd ..
EMCC_DEBUG=2
IDIRS="-Iinclude -I/home/grynca/DEV/gamedev/base/include -I/home/grynca/DEV/gamedev/assets/include -I/home/grynca/DEV/libs/emscripten/glm -I/home/grynca/DEV/gamedev/maths/include "
LDIRS=""
LIBS="-lGLEW -lGL -lGLU"
FLAGS="-std=c++11 -DWEB -DGLM_FORCE_RADIANS $IDIRS -s USE_SDL=2 -s USE_SDL_IMAGE=2 -s USE_LIBPNG=1 -s TOTAL_MEMORY=134217728 --use-preload-plugins --preload-file data -DVISUAL_DEBUG"
FLAGS+=" -DNDEBUG -O3"
TEST_SOURCES="test/main.cpp"

emcc $TEST_SOURCES $FLAGS -o build/Emscripten/test.bc

emcc build/Emscripten/test.bc $FLAGS $LDIRS $LIBS -o build/Emscripten/main.html

