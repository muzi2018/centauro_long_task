execute_process(COMMAND "/home/wang/forest_ws/src/centauro_long_task/build/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/wang/forest_ws/src/centauro_long_task/build/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
