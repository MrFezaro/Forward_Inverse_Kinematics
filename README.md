# Forward- and Inverse Kinematics of a 3-DOF arm
#### Candidate Number: 10053

![Demo](docs/assets/Program_demo.gif)
Full demo video [here](https://www.youtube.com/watch?v=q4ilPCmvRKg&ab_channel=FezaroGaming)

## Overview
This project aims to simulate the forward and inverse kinematics of a 3-DOF (Degree of Freedom) robotic arm using the threepp library. It provides a visual representation and control interface for manipulating the arm's joints and links.

## Features
- **Forward Kinematics:** Calculate the end effector position based on given joint angles.
- **Inverse Kinematics:** Determine the required joint angles to achieve a desired end effector position.
- **Dynamic Link Length:** Change the length of the links to observe the effect on the arm's movement.
- **Interactive UI:** Use ImGui for real-time control and visualization of the kinematics.
- **3D Visualization:** Render the arm and its joints in a 3D environment.
- **Cross-Platform:** Support for Windows, macOS, and Linux.

## Usage
To use this project, follow these steps:

1. **Clone the Repository:**
   ```sh
   git clone https://github.com/MrFezaro/Forward-Inverse-Kinematics.git
   cd Forward-Inverse-Kinematics
   ```
2. **Build the Project:**
   ```sh
    mkdir build
    cd build
    cmake ..
    make
    ```
3. **Run the Application:**
    ```sh
    ./kinematics_demo
    ```
4. **Control the Arm:**
    - Use the switch to toggle between forward and inverse kinematics.
    - Drag the sliders to adjust the joint angles og target position.
    - Click reset to set the arm to its initial position.

## Dependencies
- **threepp:** A C++ library for 3D graphics
    (https://github.com/markaren/threepp)
- **ImGui:** A bloat-free graphical user interface for C++
    (https://github.com/ocornut/imgui)
- **GLFW:** A library for creating windows with OpenGL contexts
    (https://www.glfw.org/)

## UML Class Diagram
![UML Diagram](UML_Diagram.png)

## Main Scripts
- **main.cpp:** Entry point of the application, sets up the scene and handles user interaction through ImGui.
- **Kinematics.cpp:** Contains the implementation of the forward and inverse kinematics algorithms.

## Installation Instructions
Make sure to have the following installed:
- CMake 3.19 or later
- A C++20 compatible compiler

## Additional Information
The project includes test functions which can be enabled by setting `PROJECT_BUILD_TESTS` to `ON` in the `CMakeLists.txt` file.
