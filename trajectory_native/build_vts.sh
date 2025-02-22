#!/bin/sh
apk add --update clang make cmake grpc-dev protobuf-dev nlohmann-json fmt-dev ninja git

CC=clang
CXX=clang++

mkdir build
cd build
cmake .. -G Ninja
cmake --build . --parallel $(nproc)
cd ..
