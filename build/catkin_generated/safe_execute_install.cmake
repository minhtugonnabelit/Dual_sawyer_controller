execute_process(COMMAND "/home/anhminhtu/sawyer_ws/src/dual_sawyer_controller/build/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/anhminhtu/sawyer_ws/src/dual_sawyer_controller/build/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
