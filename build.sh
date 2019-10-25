#!/bin/bash

mkdir -p build
cd build

cmake -DCMAKE_BUILD_TYPE=Release \
-DCMAKE_C_COMPILER=/usr/bin/clang-7 \
-DCMAKE_CXX_COMPILER=/usr/bin/clang++-7 \
-DCMAKE_LINKER=/usr/bin/ld.gold .. 
cmake --build . --target all -- -j