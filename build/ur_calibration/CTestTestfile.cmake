# CMake generated Testfile for 
# Source directory: /home/wjc/zrz_ros2_ws/src/ros2_ws/ur_calibration
# Build directory: /home/wjc/zrz_ros2_ws/src/ros2_ws/build/ur_calibration
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(calibration_test "/usr/bin/python3" "-u" "/opt/ros/jazzy/share/ament_cmake_test/cmake/run_test.py" "/home/wjc/zrz_ros2_ws/src/ros2_ws/build/ur_calibration/test_results/ur_calibration/calibration_test.gtest.xml" "--package-name" "ur_calibration" "--output-file" "/home/wjc/zrz_ros2_ws/src/ros2_ws/build/ur_calibration/ament_cmake_gmock/calibration_test.txt" "--command" "/home/wjc/zrz_ros2_ws/src/ros2_ws/build/ur_calibration/calibration_test" "--gtest_output=xml:/home/wjc/zrz_ros2_ws/src/ros2_ws/build/ur_calibration/test_results/ur_calibration/calibration_test.gtest.xml")
set_tests_properties(calibration_test PROPERTIES  LABELS "gmock" REQUIRED_FILES "/home/wjc/zrz_ros2_ws/src/ros2_ws/build/ur_calibration/calibration_test" TIMEOUT "60" WORKING_DIRECTORY "/home/wjc/zrz_ros2_ws/src/ros2_ws/build/ur_calibration" _BACKTRACE_TRIPLES "/opt/ros/jazzy/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/jazzy/share/ament_cmake_gmock/cmake/ament_add_gmock_test.cmake;98;ament_add_test;/opt/ros/jazzy/share/ament_cmake_gmock/cmake/ament_add_gmock.cmake;90;ament_add_gmock_test;/home/wjc/zrz_ros2_ws/src/ros2_ws/ur_calibration/CMakeLists.txt;84;ament_add_gmock;/home/wjc/zrz_ros2_ws/src/ros2_ws/ur_calibration/CMakeLists.txt;0;")
subdirs("gmock")
subdirs("gtest")
