# Install script for directory: /home/db/Robotics/ROS_WORKSPACE/ompl_ws/src/ompl/demos

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE PROGRAM FILES "/home/db/Robotics/ROS_WORKSPACE/ompl_ws/src/ompl/cmake-build-debug/share/ompl/demos/KinematicChainPathPlot.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE PROGRAM FILES "/home/db/Robotics/ROS_WORKSPACE/ompl_ws/src/ompl/cmake-build-debug/share/ompl/demos/OptimalPlanning.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE PROGRAM FILES "/home/db/Robotics/ROS_WORKSPACE/ompl_ws/src/ompl/cmake-build-debug/share/ompl/demos/PlannerData.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE PROGRAM FILES "/home/db/Robotics/ROS_WORKSPACE/ompl_ws/src/ompl/cmake-build-debug/share/ompl/demos/Point2DPlanning.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE PROGRAM FILES "/home/db/Robotics/ROS_WORKSPACE/ompl_ws/src/ompl/cmake-build-debug/share/ompl/demos/RandomWalkPlanner.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE PROGRAM FILES "/home/db/Robotics/ROS_WORKSPACE/ompl_ws/src/ompl/cmake-build-debug/share/ompl/demos/RigidBodyPlanning.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE PROGRAM FILES "/home/db/Robotics/ROS_WORKSPACE/ompl_ws/src/ompl/cmake-build-debug/share/ompl/demos/RigidBodyPlanningWithControls.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE PROGRAM FILES "/home/db/Robotics/ROS_WORKSPACE/ompl_ws/src/ompl/cmake-build-debug/share/ompl/demos/RigidBodyPlanningWithODESolverAndControls.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE PROGRAM FILES "/home/db/Robotics/ROS_WORKSPACE/ompl_ws/src/ompl/cmake-build-debug/share/ompl/demos/StateSampling.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE FILE FILES
    "/home/db/Robotics/ROS_WORKSPACE/ompl_ws/src/ompl/demos/CForestCircleGridBenchmark.cpp"
    "/home/db/Robotics/ROS_WORKSPACE/ompl_ws/src/ompl/demos/Diagonal.cpp"
    "/home/db/Robotics/ROS_WORKSPACE/ompl_ws/src/ompl/demos/GeometricCarPlanning.cpp"
    "/home/db/Robotics/ROS_WORKSPACE/ompl_ws/src/ompl/demos/HybridSystemPlanning.cpp"
    "/home/db/Robotics/ROS_WORKSPACE/ompl_ws/src/ompl/demos/HypercubeBenchmark.cpp"
    "/home/db/Robotics/ROS_WORKSPACE/ompl_ws/src/ompl/demos/KinematicChainBenchmark.cpp"
    "/home/db/Robotics/ROS_WORKSPACE/ompl_ws/src/ompl/demos/LTLWithTriangulation.cpp"
    "/home/db/Robotics/ROS_WORKSPACE/ompl_ws/src/ompl/demos/OpenDERigidBodyPlanning.cpp"
    "/home/db/Robotics/ROS_WORKSPACE/ompl_ws/src/ompl/demos/OptimalPlanning.cpp"
    "/home/db/Robotics/ROS_WORKSPACE/ompl_ws/src/ompl/demos/PlannerData.cpp"
    "/home/db/Robotics/ROS_WORKSPACE/ompl_ws/src/ompl/demos/PlannerProgressProperties.cpp"
    "/home/db/Robotics/ROS_WORKSPACE/ompl_ws/src/ompl/demos/Point2DPlanning.cpp"
    "/home/db/Robotics/ROS_WORKSPACE/ompl_ws/src/ompl/demos/RigidBodyPlanning.cpp"
    "/home/db/Robotics/ROS_WORKSPACE/ompl_ws/src/ompl/demos/RigidBodyPlanningWithControls.cpp"
    "/home/db/Robotics/ROS_WORKSPACE/ompl_ws/src/ompl/demos/RigidBodyPlanningWithIK.cpp"
    "/home/db/Robotics/ROS_WORKSPACE/ompl_ws/src/ompl/demos/RigidBodyPlanningWithIntegrationAndControls.cpp"
    "/home/db/Robotics/ROS_WORKSPACE/ompl_ws/src/ompl/demos/RigidBodyPlanningWithODESolverAndControls.cpp"
    "/home/db/Robotics/ROS_WORKSPACE/ompl_ws/src/ompl/demos/StateSampling.cpp"
    "/home/db/Robotics/ROS_WORKSPACE/ompl_ws/src/ompl/demos/ThunderLightning.cpp"
    "/home/db/Robotics/ROS_WORKSPACE/ompl_ws/src/ompl/demos/TriangulationDemo.cpp"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE DIRECTORY FILES "/home/db/Robotics/ROS_WORKSPACE/ompl_ws/src/ompl/demos/Koules")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE DIRECTORY FILES "/home/db/Robotics/ROS_WORKSPACE/ompl_ws/src/ompl/demos/VFRRT")
endif()

