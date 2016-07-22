#!/bin/bash
mkdir -p build
cd build
mkdir -p Emscripten
cd ..
EMCC_DEBUG=2
IDIRS="-Isrc -I/home/grynca/DEV/gamedev/base/src -I/home/grynca/DEV/gamedev/maths/src -I/home/grynca/DEV/libs/glm"
LDIRS=""
LIBS=""
FLAGS="-std=c++11 -DWEB -DGLM_FORCE_RADIANS $IDIRS"
FLAGS+=" -O3"
TEST_SOURCES="main.cpp"

emcc $TEST_SOURCES $FLAGS -o build/Emscripten/test.bc

emcc build/Emscripten/test.bc $FLAGS $LDIRS $LIBS -o build/Emscripten/main.html
