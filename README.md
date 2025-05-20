# 🤖🚗 AI-Autonomous Vehicles and Robotics

This project demonstrates the development of an intelligent autonomous system that combines *Artificial Intelligence, **Robotics, **IoT sensor integration, and **Event-Based Programming Logic (EBPL)* to enable self-driving vehicles and robots with real-time decision-making, navigation, and adaptive control.

---

## ✨ Features

- *Autonomous Navigation*: Real-time route planning and path-following using AI logic.
- *Event-Based Programming Logic (EBPL)*: Modular control flow for responsive, dynamic behavior.
- *Sensor Fusion*: Integration of LIDAR, GPS, camera, and ultrasonic data.
- *Real-Time Object & Obstacle Detection*: AI models detect traffic signs and road conditions with 92%+ accuracy.
- *IoT Integration*: Continuous data streaming from onboard sensors.
- *Security*: AES-256 encryption and RBAC for secure access.
- *Live Dashboard*: Interface for monitoring, route input, and manual override.

---

## 💻 Technology Used

- *Programming Languages*: Python, C++
- *Libraries & Frameworks*:  
  - OpenCV  
  - TensorFlow / PyTorch  
  - NumPy, Pandas  
  - EBPL Custom Framework  
  - Scikit-learn
- *Hardware & Platforms*:  
  - LIDAR, Ultrasonic Sensors, GPS Module  
  - Raspberry Pi / Embedded Controller  
  - ROS (optional for simulation)  
  - CARLA Simulator (for testing)
- *Security*: AES-256 encryption, RBAC, real-time anomaly detection

---

## ⚙ How It Works

1. *Startup*: Sensors and AI models are initialized using EBPL.
2. *Sensor Input*: Data from LIDAR, ultrasonic, GPS, and camera modules.
3. *Processing*: AI models perform object detection, lane detection, and decision-making.
4. *Event Handling*: EBPL core processes sensor events and triggers appropriate actions.
5. *Navigation*: Robot/vehicle moves based on AI-guided instructions and event triggers.
6. *User Interface*: Live dashboard displays status and allows interaction or override.

---

## 🗂 Data Collection

- *Sources*:
  - KITTI Dataset
  - Udacity Self-Driving Car Dataset
  - Custom-collected video and sensor logs
- *Labeling Tools*:
  - CVAT
  - LabelImg
- *Real-Time Data*: Captured via on-board IoT sensors during simulation/testing.
-
