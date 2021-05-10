cmake_minimum_required(VERSION 3.19)
project(emul6502)
set(CMAKE_C_STANDARD 11)
add_executable(emul6502 src/main.c src/cpu6502.c src/cpu6502.h)