
---

# Semi-Autonomous Drone Navigation 

This project explores **semi-autonomous navigation of a TurtleBot and drone using computer vision and ROS2**.
It was developed as part of the **ST5 Lab at CentraleSupélec – Université Paris-Saclay (October 2023)** by **Kiyoshi Frade Araki** and **Mauricio Orenbuch Hendel**.

---

## Overview

The main objective of this work was to implement **visual-based navigation** by detecting the **vanishing point** of a corridor through computer vision and using that information to **control a mobile robot (TurtleBot)** semi-autonomously.

The project was developed in **ROS2**, using a combination of:

* **Custom image processing nodes** for vanishing point detection.
* **Teleoperation via joystick** for manual control.
* **High-level automatic control** based on vision feedback.
* **SLAM (Simultaneous Localization and Mapping)** and **Navigation Stack** for autonomous movement in mapped environments.

---

## Project Structure

### 1. `vanishing_point` package

Handles **computer vision processing** using OpenCV’s Line Segment Detector (LSD) to:

* Detect dominant lines in the image.
* Compute the **vanishing point** (intersection density of lines).
* Publish visual indicators such as:

  * **Horizontal misplacement** (how far the vanishing point is from the image center)
  * **Angle unbalance** (difference between two perspective angles).

### 2. `vanishing_turtlebot` package

Provides **integration between vision and robot control**:

* A **teleoperation node** allows manual control using a joystick.
* An **automatic controller node** adjusts the TurtleBot’s trajectory based on the detected vanishing point.
* Config and launch files to start the complete system easily.

---

## Manual Teleoperation

A dedicated node (`joy_teleop`) lets the operator control the TurtleBot’s **linear and angular velocity** using a joystick.
A “deadman button” ensures safety by sending velocity commands only when pressed.

---

##  Vanishing Point Detection

The algorithm filters image lines, detects those with **opposite normals**, and calculates the most stable intersection point (the **vanishing point**) using:

* A **KNN density filter** to find the most consistent intersection region.
* A **memory effect**, keeping the vanishing point stable over time.

It outputs:

* `vp_offset`: normalized horizontal misplacement (`-1` = left, `+1` = right)
* `vp_angle`: unbalance between perspective angles.

**Example results:**

| TurtleBot Corridor | Drone Corridor   |
| ------------------ | ---------------- |
| ![vp1](images\vp1.jpeg)   | ![vp2](images\vp2.jpeg) |

---

## SLAM and Navigation

After testing visual control, we extended the project using **ROS2 SLAM Toolbox** and **Navigation Stack** to enable:

* Real-time mapping of unknown environments.
* Autonomous path planning and obstacle avoidance.
* Manual override priority (joystick commands take precedence via twist multiplexer).

**Example RViz map:**

![SLAM Map](images\slam.png)

---

## Key Concepts

* **Vanishing Point:** The intersection point of parallel lines in perspective; used here as a reference for heading alignment.
* **Visual Servoing:** Adjusting robot velocity based on image feedback.
* **SLAM:** Simultaneous localization and mapping for autonomous navigation.

---

## Demo and Source

**Video Demonstration:** [https://youtu.be/_FsErYMsxq8w](https://youtu.be/_FsErYMsxq8w)

---

## Technologies Used

* **ROS2 (Humble/Foxy)**
* **Python 3**
* **OpenCV**
* **TurtleBot3**
* **RViz2 / Gazebo**
* **SLAM Toolbox**
* **Twist Mux**

---

##  Authors

**Kiyoshi Frade Araki**
**Mauricio Orenbuch Hendel**
*CentraleSupélec – Université Paris-Saclay, October 2023*

