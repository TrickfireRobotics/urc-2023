cmake_minimum_required(VERSION 3.20)
project(your_project)

find_package(myactuator_rmd REQUIRED)

add_executable(your_node
  main.cpp
)
target_link_libraries(your_node PUBLIC myactuator_rmd::myactuator_rmd)