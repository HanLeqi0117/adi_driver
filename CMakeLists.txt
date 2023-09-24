cmake_minimum_required(VERSION 3.8)
project(adi_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Nodeの実行ファイルをビルドする
add_executable(adis16465_node src/adis16465_node.cpp src/adis16465.cpp)
add_executable(adis16470_node src/adis16470_node.cpp src/adis16470.cpp)

# Nodeのコードをビルドする際に、依頼するヘッドファイルのディレクトリを記入する
target_include_directories(adis16465_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# ビルドするC言語及びC++言語のバージョンを選択する
target_compile_features(adis16465_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# amentビルドで依頼するROS2のパッケージを記入する
ament_target_dependencies(
  adis16465_node
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "std_srvs"
)

target_include_directories(adis16470_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(adis16470_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  adis16470_node
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "std_srvs"
)

# 実行ファイルの生成
install(TARGETS adis16465_node adis16470_node
  DESTINATION lib/${PROJECT_NAME})

# installディレクトリ下にlaunchとconfigディレクトリを追加する(中のファイルもコピーされる)
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
