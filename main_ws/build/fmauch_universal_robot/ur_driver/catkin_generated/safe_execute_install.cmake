execute_process(COMMAND "/home/bspetersson/Documents/P5_Lego_Throwing_Robot/Code/Lego-Throwing-Robot/main_ws/build/fmauch_universal_robot/ur_driver/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/bspetersson/Documents/P5_Lego_Throwing_Robot/Code/Lego-Throwing-Robot/main_ws/build/fmauch_universal_robot/ur_driver/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
