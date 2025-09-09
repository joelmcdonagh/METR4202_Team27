# CMake generated Testfile for 
# Source directory: /home/kilian/arCuo_scanning/opencv-4.x/modules/core
# Build directory: /home/kilian/arCuo_scanning/build/modules/core
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(opencv_test_core "/home/kilian/arCuo_scanning/build/bin/opencv_test_core" "--gtest_output=xml:opencv_test_core.xml")
set_tests_properties(opencv_test_core PROPERTIES  LABELS "Main;opencv_core;Accuracy" WORKING_DIRECTORY "/home/kilian/arCuo_scanning/build/test-reports/accuracy" _BACKTRACE_TRIPLES "/home/kilian/arCuo_scanning/opencv-4.x/cmake/OpenCVUtils.cmake;1799;add_test;/home/kilian/arCuo_scanning/opencv-4.x/cmake/OpenCVModule.cmake;1365;ocv_add_test_from_target;/home/kilian/arCuo_scanning/opencv-4.x/modules/core/CMakeLists.txt;212;ocv_add_accuracy_tests;/home/kilian/arCuo_scanning/opencv-4.x/modules/core/CMakeLists.txt;0;")
add_test(opencv_perf_core "/home/kilian/arCuo_scanning/build/bin/opencv_perf_core" "--gtest_output=xml:opencv_perf_core.xml")
set_tests_properties(opencv_perf_core PROPERTIES  LABELS "Main;opencv_core;Performance" WORKING_DIRECTORY "/home/kilian/arCuo_scanning/build/test-reports/performance" _BACKTRACE_TRIPLES "/home/kilian/arCuo_scanning/opencv-4.x/cmake/OpenCVUtils.cmake;1799;add_test;/home/kilian/arCuo_scanning/opencv-4.x/cmake/OpenCVModule.cmake;1264;ocv_add_test_from_target;/home/kilian/arCuo_scanning/opencv-4.x/modules/core/CMakeLists.txt;213;ocv_add_perf_tests;/home/kilian/arCuo_scanning/opencv-4.x/modules/core/CMakeLists.txt;0;")
add_test(opencv_sanity_core "/home/kilian/arCuo_scanning/build/bin/opencv_perf_core" "--gtest_output=xml:opencv_perf_core.xml" "--perf_min_samples=1" "--perf_force_samples=1" "--perf_verify_sanity")
set_tests_properties(opencv_sanity_core PROPERTIES  LABELS "Main;opencv_core;Sanity" WORKING_DIRECTORY "/home/kilian/arCuo_scanning/build/test-reports/sanity" _BACKTRACE_TRIPLES "/home/kilian/arCuo_scanning/opencv-4.x/cmake/OpenCVUtils.cmake;1799;add_test;/home/kilian/arCuo_scanning/opencv-4.x/cmake/OpenCVModule.cmake;1265;ocv_add_test_from_target;/home/kilian/arCuo_scanning/opencv-4.x/modules/core/CMakeLists.txt;213;ocv_add_perf_tests;/home/kilian/arCuo_scanning/opencv-4.x/modules/core/CMakeLists.txt;0;")
