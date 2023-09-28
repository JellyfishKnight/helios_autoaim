cmake_minimum_required(VERSION 3.22.0)
project(helios_autoaim)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Ceres CONFIG REQUIRED)
find_package(OpenVINO REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

generate_parameter_library(
  helios_autoaim_parameters # cmake target name for the parameter library
  src/helios_autoaim_parameters.yaml # path to input yaml file
)

target_include_directories(
  ${PROJECT_NAME} PUBLIC
  include/
  include/detectors
  include/predictors
  include/utilities
)

target_link_libraries(
  ${PROJECT_NAME}
  ${OpenCV_LIBS}
  Eigen3::Eigen
  ceres Ceres::ceres
  helios_autoaim_parameters
  openvino::runtime
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN helios_cv::HeliosAutoAim
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING) 
  find_package(ament_lint_auto REQUIRED)
  list(
    APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  model
)