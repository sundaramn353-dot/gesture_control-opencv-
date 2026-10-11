# Gesture-Controlled Robot Simulation Using ROS 2

## 📌 Project Overview

This project implements a **hand gesture-based robot control system** using ROS 2, OpenCV, and MediaPipe. The robot is controlled in a Gazebo simulation environment by detecting the number of fingers shown by the user through a webcam.

The system recognizes hand gestures in real time and sends the corresponding movement commands to the simulated robot.

##  Technologies Used

- **ROS 2** – Robot communication and control
- **Python** – Gesture recognition and control logic
- **OpenCV** – Webcam capture and image processing
- **MediaPipe** – Real-time hand tracking and finger detection
- **Gazebo** – Robot simulation and virtual world

##  Hand Gesture Controls

| Hand Gesture | Robot Action | Movement |
|---|---|---|
| ☝️ One finger | Move Forward | Positive X |
| ✌️ Two fingers | Move Backward | Negative X |
| 🤟 Three fingers | Turn Left | Negative Angular Z |
| 🖖 Four fingers | Turn Right | Positive Angular Z |
| 🖐️ Five fingers | Stop | Zero Linear and Angular Velocity |

##  How It Works

1. The webcam captures the user's hand gestures.
2. OpenCV processes the camera frames.
3. MediaPipe detects and tracks the hand landmarks.
4. The finger-counting logic identifies the gesture.
5. The ROS 2 node publishes the corresponding velocity commands using the `geometry_msgs/msg/Twist` message.
6. The simulated robot receives the commands and moves inside the Gazebo world.

##  Project Objective

The main objective is to develop a natural and intuitive human-robot interaction system that allows users to control a simulated mobile robot using simple hand gestures, without a keyboard or joystick.

##  Future Improvements

- Add more hand gestures for additional robot actions.
- Improve gesture detection accuracy and robustness.
- Add obstacle avoidance and autonomous navigation.
- Extend gesture-based control to a physical mobile robot.

##  Author

Developed as a ROS 2 robotics project exploring computer vision, hand gesture recognition, and robot simulation.
