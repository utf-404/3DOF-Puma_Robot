cmake_minimum_required(VERSION 3.8)
project(puma_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 패키지 의존성 찾기
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# URDF 및 Launch 파일 설치 (해당 폴더가 존재하는지 확인 후 추가)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/urdf")
  install(DIRECTORY urdf
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# Python 실행 파일 설치
install(
  PROGRAMS
  scripts/puma_controller.py
  scripts/ros2_web_bridge.py
  scripts/joint_state_logger.py  
  scripts/test.py
  DESTINATION lib/${PROJECT_NAME}
)

# 테스트 활성화 (필요할 경우)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
