# CMake generated Testfile for 
# Source directory: /home/kilian/arCuo_scanning/opencv-4.x/modules/gapi
# Build directory: /home/kilian/arCuo_scanning/build/modules/gapi
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(opencv_test_gapi "/home/kilian/arCuo_scanning/build/bin/opencv_test_gapi" "--gtest_output=xml:opencv_test_gapi.xml")
set_tests_properties(opencv_test_gapi PROPERTIES  LABELS "Main;opencv_gapi;Accuracy" WORKING_DIRECTORY "/home/kilian/arCuo_scanning/build/test-reports/accuracy" _BACKTRACE_TRIPLES "/home/kilian/arCuo_scanning/opencv-4.x/cmake/OpenCVUtils.cmake;1799;add_test;/home/kilian/arCuo_scanning/opencv-4.x/cmake/OpenCVModule.cmake;1365;ocv_add_test_from_target;/home/kilian/arCuo_scanning/opencv-4.x/modules/gapi/CMakeLists.txt;292;ocv_add_accuracy_tests;/home/kilian/arCuo_scanning/opencv-4.x/modules/gapi/CMakeLists.txt;0;")
add_test(opencv_perf_gapi "/home/kilian/arCuo_scanning/build/bin/opencv_perf_gapi" "--gtest_output=xml:opencv_perf_gapi.xml")
set_tests_properties(opencv_perf_gapi PROPERTIES  LABELS "Main;opencv_gapi;Performance" WORKING_DIRECTORY "/home/kilian/arCuo_scanning/build/test-reports/performance" _BACKTRACE_TRIPLES "/home/kilian/arCuo_scanning/opencv-4.x/cmake/OpenCVUtils.cmake;1799;add_test;/home/kilian/arCuo_scanning/opencv-4.x/cmake/OpenCVModule.cmake;1264;ocv_add_test_from_target;/home/kilian/arCuo_scanning/opencv-4.x/modules/gapi/CMakeLists.txt;410;ocv_add_perf_tests;/home/kilian/arCuo_scanning/opencv-4.x/modules/gapi/CMakeLists.txt;0;")
add_test(opencv_sanity_gapi "/home/kilian/arCuo_scanning/build/bin/opencv_perf_gapi" "--gtest_output=xml:opencv_perf_gapi.xml" "--perf_min_samples=1" "--perf_force_samples=1" "--perf_verify_sanity")
set_tests_properties(opencv_sanity_gapi PROPERTIES  LABELS "Main;opencv_gapi;Sanity" WORKING_DIRECTORY "/home/kilian/arCuo_scanning/build/test-reports/sanity" _BACKTRACE_TRIPLES "/home/kilian/arCuo_scanning/opencv-4.x/cmake/OpenCVUtils.cmake;1799;add_test;/home/kilian/arCuo_scanning/opencv-4.x/cmake/OpenCVModule.cmake;1265;ocv_add_test_from_target;/home/kilian/arCuo_scanning/opencv-4.x/modules/gapi/CMakeLists.txt;410;ocv_add_perf_tests;/home/kilian/arCuo_scanning/opencv-4.x/modules/gapi/CMakeLists.txt;0;")
