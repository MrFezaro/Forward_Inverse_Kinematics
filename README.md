# Forward and Inverse Kinematics of a 3-DOF Arm

#### Candidate Number: 10053

<div style="text-align: center;">
  <img src="docs/assets/Program_demo.gif" alt="Demo">
  <p>Full demo video <a href="https://www.youtube.com/watch?v=q4ilPCmvRKg&ab_channel=FezaroGaming">here</a></p>
</div>

## Overview

This project aims to simulate the forward and inverse kinematics of a 3-DOF (Degree of Freedom) robotic arm using the
threepp library. It provides a visual representation and control interface for manipulating the arm's joints and links.

## Features

- **Forward Kinematics:** Calculate the end effector position based on given joint angles.
- **Inverse Kinematics:** Determine the required joint angles to achieve a desired end effector position using cyclic coordinate decent.
- **Dynamic Link Length:** Change the length of the links to observe the effect on the arm's movement.
- **Interactive UI:** Use ImGui for real-time control and visualization of the kinematics.
- **3D Visualization:** Render the arm and its joints in a 3D environment.
- **Cross-Platform:** Support for Windows, macOS, and Linux.

## Usage

- Use the switch to toggle between forward and inverse kinematics.
- Drag the sliders to adjust the joint angles og target position.
- Click reset to set the arm to its initial position.
- Use the drop down menu in inverse kinematics mode to play different animations.

## Dependencies

- **threepp:** A C++ library for 3D graphics
  (https://github.com/markaren/threepp)
- **ImGui:** A graphical user interface for C++
  (https://github.com/ocornut/imgui)

## UML Class Diagram

![UML Diagram](docs/assets/UML_Diagram.jpg)

## Main Scripts

- **main.cpp:** Entry point of the application, sets up the scene and handles user interaction through ImGui.
- **testFunctions.cpp:**  Tests different functions for the chain kinematics class.

## Additional Information

Make sure to have the following installed:

- CMake 3.19 or later
- A C++20 compatible compiler

The test functions can be disabled by setting `PROJECT_BUILD_TESTS` to `OFF` in the
`CMakeLists.txt` file.
