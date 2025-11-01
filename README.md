Simple Object Detection and Following Robot (in Simulation)

[Placeholder: GIF/Video demonstrating TurtleBot3 autonomously tracking the target object in Gazebo]

This project demonstrates Visual Servoing—the use of visual information (from a camera) to control a robot's movement—within a complete ROS/Gazebo simulation environment. It is a foundational exercise in applying Deep Learning (AI/ML) to classic mobile robotics control problems.

🌟 Core Technical Contributions

Component

Description

Technologies

Perception Pipeline

Developed a Python ROS node to receive simulated camera images, process them using a lightweight YOLO-nano model for real-time object detection, and calculate the object's positional error (offset from the center).

ROS, Python, YOLO, OpenCV, cv_bridge

Autonomous Control

Implemented a dedicated ROS Proportional (P) Controller node that reads the positional error and generates precise linear and angular Twist commands to drive the robot toward the target object autonomously.

ROS, Control Theory (P-Control), geometry_msgs

Simulation & Integration

Structured the system using ROS launch files, topics, and configuration files to enable seamless operation of the mobile robot (TurtleBot3) in the Gazebo environment.

ROS, Gazebo, TurtleBot3, package.xml

📐 ROS System Architecture

The project is structured into modular nodes for scalability and clear separation of concerns (Perception vs. Control).

Data Flow Diagram

graph LR
    A[Gazebo /camera/image_raw] --> B(ai_perception_node: YOLO);
    B --> C[/object_detection/error];
    C --> D(p_control_node: P-Controller);
    D --> E[/cmd_vel];
    E --> F[TurtleBot3];


Data Flow Overview

/camera/image_raw: Image data is streamed from the Gazebo camera.

ai_perception_node: Subscribes to the image, detects the object, and publishes the offset error.

/object_detection/error: A custom topic carrying the X-axis (turning) and Z-axis (distance) errors.

p_control_node: Subscribes to the error and applies P-Control to calculate velocities.

/cmd_vel: The robot subscribes here to execute movement commands.

⚙️ Repository Structure

vision_follower/
├── src/
│   ├── ai_perception_node.py  # AI/YOLO Integration (Perception)
│   └── p_control_node.py      # P-Controller Logic (Control)
├── launch/
│   └── follower.launch        # ROS Launch file for entire system
├── config/
│   └── p_gains.yaml           # Tunable P-Controller Gains
├── package.xml                # ROS Manifest & Dependencies
└── CMakeLists.txt             # ROS Build Script



⚠️ Status: Active Development

This project is currently under active development. The core ROS architecture and foundational control logic are complete, and the AI/YOLO integration is being finalized for a robust real-time performance demonstration.
