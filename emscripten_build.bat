mkdir build
cd build
mkdir Emscripten
cd ..
set EMCC_DEBUG=2

set IDIRS= -Isrc -Iinclude -Ic:/DEV/gamedev/base/src -Ic:/DEV/gamedev/maths/src -Ic:/DEV/gamedev/assets/include -Ic:/DEV/libs/mingw/glm
set FLAGS= -std=c++11 -DNDEBUG -DUSE_SDL2 -DWEB -DGLM_FORCE_RADIANS %IDIRS% -s USE_SDL=2 -s USE_SDL_IMAGE=2 -s USE_LIBPNG=1 -s TOTAL_MEMORY=134217728 -O3 --use-preload-plugins --preload-file data

set SOURCES= src/main.cpp src/update_loop.cpp

call emcc %SOURCES% %FLAGS% -o build/Emscripten/main.bc

call emcc build/Emscripten/main.bc %FLAGS% -o build/Emscripten/main.html
