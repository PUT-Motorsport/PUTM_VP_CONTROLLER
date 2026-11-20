cmake_minimum_required(VERSION 3.8)
project(putm_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(putm_vcl_interfaces REQUIRED)
find_package(vectornav_msgs REQUIRED)

add_executable(controller src/controller.cpp src/putm_sim_gen/tv_code.c
                          src/putm_sim_gen/tv_code_data.c src/putm_sim_gen/read.c src/putm_sim_gen/rtGetInf.c
                          src/putm_sim_gen/rtGetNaN.c src/putm_sim_gen/rt_nonfinite.c)
target_include_directories(
  controller
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/putm_sim_gen>)

ament_target_dependencies(controller rclcpp putm_vcl_interfaces vectornav_msgs)
target_compile_features(controller PUBLIC c_std_99 cxx_std_17)

install(TARGETS controller DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
