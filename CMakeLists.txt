cmake_minimum_required(VERSION 3.8)
project(putm_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(putm_vcl_interfaces REQUIRED)
find_package(putm_vcl REQUIRED)
find_package(vectornav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(controller src/controller.cpp src/putm_sim_gen/tv_code.c
                          src/putm_sim_gen/read.c src/putm_sim_gen/rt_nonfinite.c)
target_include_directories(
  controller
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/putm_sim_gen>)

ament_target_dependencies(controller rclcpp putm_vcl_interfaces putm_vcl vectornav_msgs)
target_link_libraries(controller Eigen3::Eigen)
target_compile_features(controller PUBLIC c_std_99 cxx_std_17)

install(TARGETS controller DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
