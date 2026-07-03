cmake_minimum_required(VERSION 3.5)
project(ms_yolo_msg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === 必要な依存パッケージ ===
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

# === メッセージの生成 ===
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StateYolo.msg"
  DEPENDENCIES std_msgs  # ← ここが重要
)

# === インストール設定（省略されがちですが必要）===
ament_export_dependencies(rosidl_default_runtime)
ament_package()
