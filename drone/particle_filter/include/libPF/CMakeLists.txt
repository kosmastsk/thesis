project(libPF)
cmake_minimum_required(VERSION 2.8)

include_directories(include)

add_library(PF
  src/CRandomNumberGenerator.cpp)
